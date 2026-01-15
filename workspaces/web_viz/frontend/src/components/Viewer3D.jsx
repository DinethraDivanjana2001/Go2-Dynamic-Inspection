import React, { useEffect, useRef, useState, useMemo } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Line, Html } from '@react-three/drei';
import * as THREE from 'three';

// --- Configuration ---
const VOXEL_SIZE = 0.18;
const MIN_Z = -1.0;
const MAX_Z = 2.0;

// --- Colors ---
// Dark/Blue Theme
const THEME = {
    bg: '#0f172a', // Slate 900
    grid: '#334155', // Slate 700
    accent: '#3b82f6', // Blue 500
    text: '#ffffff',
    panel: 'rgba(30, 41, 59, 0.95)', // Slate 800
};

// --- Utilities ---
function getHeatmapColor(t) {
    const clamped = Math.max(0, Math.min(1, t));
    const hue = (1.0 - clamped) * 240; // Blue to Red
    return new THREE.Color(`hsl(${hue}, 100%, 50%)`);
}

// --- Components ---

const VoxelCloud = ({ url }) => {
    const meshRef = useRef();
    const MAX_INSTANCES = 150000;
    const dummy = useMemo(() => new THREE.Object3D(), []);

    useEffect(() => {
        const ws = new WebSocket(url);
        ws.binaryType = "arraybuffer";
        ws.onmessage = (event) => {
            if (event.data instanceof ArrayBuffer && meshRef.current) {
                const floatView = new Float32Array(event.data);
                const count = Math.floor(floatView.length / 3);
                const limit = Math.min(count, MAX_INSTANCES);

                for (let i = 0; i < limit; i++) {
                    const x = floatView[i * 3];
                    const y = floatView[i * 3 + 1];
                    const z = floatView[i * 3 + 2];

                    dummy.position.set(x, y, z);
                    dummy.updateMatrix();
                    meshRef.current.setMatrixAt(i, dummy.matrix);

                    const t = (z - MIN_Z) / (MAX_Z - MIN_Z);
                    meshRef.current.setColorAt(i, getHeatmapColor(t));
                }
                meshRef.current.count = limit;
                meshRef.current.instanceMatrix.needsUpdate = true;
                if (meshRef.current.instanceColor) meshRef.current.instanceColor.needsUpdate = true;
            }
        };
        return () => ws.close();
    }, [url]);

    return (
        <instancedMesh ref={meshRef} args={[null, null, MAX_INSTANCES]}>
            <boxGeometry args={[VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE]} />
            <meshStandardMaterial />
        </instancedMesh>
    );
};

// Procedural Robot Dog (Unitree Go1 Style)
const RobotDog = ({ color = "#cbd5e1" }) => {
    // A simplified dog: Body + Head + 4 Legs
    return (
        <group>
            {/* Body */}
            <mesh position={[0, 0, 0]}>
                <boxGeometry args={[0.6, 0.25, 0.2]} />
                <meshStandardMaterial color="#475569" />
            </mesh>
            {/* Head */}
            <mesh position={[0.35, 0, 0.1]}>
                <boxGeometry args={[0.2, 0.15, 0.15]} />
                <meshStandardMaterial color="#94a3b8" />
            </mesh>
            {/* Legs (Front L/R, Back L/R) */}
            <mesh position={[0.25, 0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#334155" />
            </mesh>
            <mesh position={[0.25, -0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#334155" />
            </mesh>
            <mesh position={[-0.25, 0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#334155" />
            </mesh>
            <mesh position={[-0.25, -0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#334155" />
            </mesh>
        </group>
    )
}

const DataVisualizer = ({ url }) => {
    const [tfs, setTfs] = useState({});
    const [path, setPath] = useState([]);

    useEffect(() => {
        const ws = new WebSocket(url);
        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                if (data.tfs) setTfs(data.tfs);
                if (data.path) setPath(data.path);
            } catch (e) { }
        };
        return () => ws.close();
    }, [url]);

    const linePoints = useMemo(() => path.map(p => [p.x, p.y, p.z]), [path]);

    return (
        <group>
            {Object.entries(tfs).map(([childId, tf]) => {
                const p = [tf.translation.x, tf.translation.y, tf.translation.z];
                const q = new THREE.Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);

                // If it's the base_link, render the Robot Dog!
                if (childId.includes("base_link")) {
                    return (
                        <group key={childId} position={p} quaternion={q}>
                            <RobotDog />
                            <axesHelper args={[0.5]} />
                        </group>
                    )
                }

                if (childId.includes("livox")) {
                    return (
                        <group key={childId} position={p} quaternion={q}>
                            <axesHelper args={[0.2]} />
                        </group>
                    )
                }
                return null;
            })}

            {linePoints.length > 1 && (
                <Line points={linePoints} color={THEME.accent} lineWidth={6} />
            )}
        </group>
    );
}

const WaypointMarker = ({ wp, onClick }) => {
    return (
        <group position={[wp.x, wp.y, wp.z]}>
            <mesh onClick={(e) => { e.stopPropagation(); onClick(wp); }}>
                <sphereGeometry args={[0.2]} />
                <meshStandardMaterial color={THEME.accent} emissive={THEME.accent} emissiveIntensity={0.5} />
            </mesh>
            <Html position={[0, 0, 0.6]} center distanceFactor={12} style={{ pointerEvents: 'none' }}>
                <div className="bg-blue-600/90 text-white text-base px-4 py-1.5 rounded-full shadow-lg border border-blue-400 backdrop-blur font-bold tracking-wide transform -translate-y-6">
                    {wp.name}
                </div>
            </Html>
        </group>
    )
}

const CameraFeed = () => {
    const [imageSrc, setImageSrc] = useState(null);

    useEffect(() => {
        const ws = new WebSocket("ws://localhost:8000/ws/video");
        ws.onmessage = (event) => {
            setImageSrc(`data:image/jpeg;base64,${event.data}`);
        };
        return () => ws.close();
    }, []);

    if (!imageSrc) return (
        <div className="w-full h-48 bg-black flex items-center justify-center text-slate-500 text-xs font-mono border border-slate-700 rounded">
            NO CAMERA FEED
        </div>
    );

    return (
        <div className="relative border border-blue-500/50 rounded overflow-hidden shadow-lg bg-black">
            <img src={imageSrc} alt="Robot Camera" className="w-full h-auto object-cover" />
            <div className="absolute top-1 right-1 flex gap-1">
                <div className="w-2 h-2 bg-red-500 rounded-full animate-pulse shadow-[0_0_8px_red]"></div>
            </div>
            <div className="absolute bottom-1 left-2 text-[10px] text-blue-200 bg-black/50 px-1 rounded font-mono">
                CAM_01 :: RAW
            </div>
        </div>
    );
}

const InteractionPlane = ({ onPointSelected }) => {
    return (
        <mesh
            rotation={[-Math.PI / 2, 0, 0]}
            position={[0, 0, 0]}
            onClick={(e) => {
                e.stopPropagation();
                const worldP = e.point;
                // Transform World -> ROS
                onPointSelected({ x: worldP.x, y: -worldP.z, z: worldP.y });
            }}
        >
            <planeGeometry args={[100, 100]} />
            <meshBasicMaterial visible={false} />
        </mesh>
    );
}

const Viewer3D = ({ onBack }) => {
    const [selectedPoint, setSelectedPoint] = useState(null);
    const [waypoints, setWaypoints] = useState([]);
    const [wpName, setWpName] = useState("");

    useEffect(() => {
        const fetchWps = () => {
            fetch('http://localhost:8000/waypoints').then(res => res.json()).then(setWaypoints).catch(e => console.error(e));
        }
        fetchWps();
        const interval = setInterval(fetchWps, 2000);
        return () => clearInterval(interval);
    }, []);

    const handleNavigate = (point) => {
        fetch('http://localhost:8000/navigate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x: point.x, y: point.y, z: point.z })
        }).then(() => setSelectedPoint(null));
    };

    const handleSaveWaypoint = () => {
        if (!selectedPoint || !wpName) return;
        fetch('http://localhost:8000/waypoints', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name: wpName, x: selectedPoint.x, y: selectedPoint.y, z: selectedPoint.z })
        }).then(res => res.json()).then(data => {
            setWaypoints(data.waypoints);
            setWpName("");
            setSelectedPoint(null);
        });
    };

    const handleDeleteWaypoint = (name) => {
        fetch(`http://localhost:8000/waypoints/${name}`, { method: 'DELETE' }).then(res => res.json()).then(data => setWaypoints(data.waypoints));
    }

    return (
        <div className="w-full h-screen bg-slate-900 flex flex-col font-sans text-slate-100 overflow-hidden">
            {/* Header */}
            <header className="bg-slate-900 border-b border-blue-900/50 h-14 flex items-center justify-between px-6 shadow-2xl z-20 shrink-0">
                <div className="flex items-center gap-4">
                    <button onClick={onBack} className="p-1 hover:text-blue-400 transition-colors">
                        <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth="2" d="M10 19l-7-7m0 0l7-7m-7 7h18"></path></svg>
                    </button>
                    <div className="text-xl font-bold tracking-wider flex items-center gap-2 text-white">
                        <div className="w-6 h-6 bg-gradient-to-tr from-blue-600 to-cyan-400 rounded flex items-center justify-center text-[10px] font-black shadow-[0_0_10px_rgba(59,130,246,0.6)]">RG</div>
                        RoboticGen<span className="text-blue-500 font-thin">OS</span>
                    </div>
                </div>
                <div className="flex items-center gap-4 text-xs font-mono text-blue-300/80">
                    <span className="flex items-center gap-2"><div className="w-1.5 h-1.5 bg-green-500 rounded-full shadow-[0_0_5px_lime]"></div> ONLINE</span>
                    <span className="opacity-50">|</span>
                    <span>V.2.0.4 BUILD_AX9</span>
                </div>
            </header>

            <div className="flex flex-1 overflow-hidden relative">
                {/* 3D Viewport */}
                <div className="flex-1 relative bg-slate-950">
                    <div className="absolute top-4 left-4 z-10 bg-black/60 backdrop-blur border border-white/10 p-2 rounded text-[10px] font-mono text-blue-300">
                        LAT: 2s :: VOX_GRID [{VOXEL_SIZE}m]
                    </div>

                    <Canvas camera={{ position: [5, 5, 5], fov: 50 }} dpr={[1, 2]} shadows>
                        <color attach="background" args={[THEME.bg]} />
                        <fog attach="fog" args={[THEME.bg, 10, 50]} />

                        <ambientLight intensity={0.5} />
                        <spotLight position={[10, 20, 10]} intensity={1} castShadow shadow-mapSize={2048} />
                        <pointLight position={[-10, -10, -10]} intensity={0.5} color="#3b82f6" />

                        <gridHelper args={[60, 60, THEME.grid, THEME.grid]} />
                        <axesHelper args={[2]} />

                        <group rotation={[-Math.PI / 2, 0, 0]}>
                            <VoxelCloud url="ws://localhost:8000/ws/points" />
                            <DataVisualizer url="ws://localhost:8000/ws/tf" />
                            {waypoints.map((wp, i) => <WaypointMarker key={i} wp={wp} onClick={handleNavigate} />)}

                            {selectedPoint && (
                                <mesh position={[selectedPoint.x, selectedPoint.y, selectedPoint.z]}>
                                    <sphereGeometry args={[0.25]} />
                                    <meshBasicMaterial color="#ef4444" transparent opacity={0.6} wireframe />
                                </mesh>
                            )}
                        </group>

                        <InteractionPlane onPointSelected={setSelectedPoint} />
                        <OrbitControls makeDefault minPolarAngle={0} maxPolarAngle={Math.PI / 1.7} />
                    </Canvas>
                </div>

                {/* Right Sidebar */}
                <aside className="w-80 bg-slate-900 border-l border-white/5 z-10 flex flex-col shadow-2xl">
                    <div className="p-4 border-b border-white/5">
                        <h2 className="text-xs font-bold text-blue-400 uppercase tracking-[0.2em]">Operations</h2>
                    </div>

                    <div className="flex-1 overflow-y-auto p-4 space-y-6">
                        {/* Camera Module */}
                        <div>
                            <div className="flex justify-between items-center mb-2">
                                <h3 className="text-[10px] font-bold text-slate-500 uppercase">Live Feed</h3>
                                <div className="text-[10px] text-green-500 font-mono">15 FPS</div>
                            </div>
                            <CameraFeed />
                        </div>

                        {/* Waypoints Module */}
                        <div>
                            <div className="flex items-center justify-between mb-2">
                                <h3 className="text-[10px] font-bold text-slate-500 uppercase">Waypoints</h3>
                                <span className="text-[10px] bg-blue-900/50 text-blue-300 px-2 py-0.5 rounded border border-blue-500/20">{waypoints.length}</span>
                            </div>
                            <div className="bg-slate-800/50 border border-white/5 rounded-lg overflow-hidden">
                                {waypoints.length === 0 ? (
                                    <div className="p-4 text-center text-xs text-slate-600 font-mono">NO DATA</div>
                                ) : (
                                    <ul className="divide-y divide-white/5">
                                        {waypoints.map((wp, i) => (
                                            <li key={i} className="flex items-center justify-between p-3 hover:bg-white/5 transition-colors group">
                                                <span className="text-sm font-medium text-slate-300">{wp.name}</span>
                                                <div className="flex gap-2 opacity-0 group-hover:opacity-100 transition-opacity">
                                                    <button onClick={() => handleNavigate(wp)} className="text-xs text-blue-400 hover:text-blue-300 font-bold">GO</button>
                                                    <button onClick={() => handleDeleteWaypoint(wp.name)} className="text-xs text-red-400 hover:text-red-300">X</button>
                                                </div>
                                            </li>
                                        ))}
                                    </ul>
                                )}
                            </div>
                        </div>

                        {/* Control Module */}
                        {selectedPoint && (
                            <div className="bg-blue-900/10 border border-blue-500/30 rounded-lg p-4 animate-in fade-in slide-in-from-bottom-2">
                                <h3 className="text-[10px] font-bold text-blue-400 uppercase mb-3">Target Lock</h3>
                                <div className="grid grid-cols-2 gap-2 mb-3 font-mono text-[10px] text-slate-400">
                                    <div className="bg-black/30 p-1.5 rounded border border-white/5">X: {selectedPoint.x.toFixed(2)}</div>
                                    <div className="bg-black/30 p-1.5 rounded border border-white/5">Y: {selectedPoint.y.toFixed(2)}</div>
                                </div>
                                <div className="space-y-2">
                                    <button onClick={() => handleNavigate(selectedPoint)} className="w-full bg-blue-600 hover:bg-blue-500 text-white py-2 rounded text-xs font-bold shadow-[0_0_15px_rgba(37,99,235,0.3)] transition-all">
                                        INITIATE NAV
                                    </button>
                                    <div className="flex gap-2">
                                        <input
                                            value={wpName}
                                            onChange={e => setWpName(e.target.value)}
                                            placeholder="DESIGNATION"
                                            className="flex-1 bg-black/30 border border-white/10 rounded px-2 text-xs text-white placeholder-slate-600 focus:outline-none focus:border-blue-500"
                                        />
                                        <button onClick={handleSaveWaypoint} className="px-3 bg-slate-700 hover:bg-slate-600 text-white rounded text-xs font-bold">SV</button>
                                    </div>
                                    <button onClick={() => setSelectedPoint(null)} className="w-full text-[10px] text-slate-500 hover:text-slate-400 mt-1">CANCEL</button>
                                </div>
                            </div>
                        )}
                    </div>
                </aside>
            </div>
        </div>
    );
};

export default Viewer3D;
