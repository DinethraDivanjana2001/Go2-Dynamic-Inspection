import React, { useEffect, useRef, useState, useMemo } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Line, Html } from '@react-three/drei';
import * as THREE from 'three';
import { ChevronDown, ChevronRight, Video, MapPin, Sliders, Settings, Sun, Moon, Layout, ArrowUp } from 'lucide-react';

// --- Configuration ---
const VOXEL_SIZE = 0.18;
const MIN_Z = -1.0;
const MAX_Z = 2.0;

// --- Themes ---
const THEMES = {
    dark: {
        name: 'Dark Blue',
        bg: '#0f172a', // Slate 900
        grid: '#334155', // Slate 700
        accent: '#3b82f6', // Blue 500
        text: '#ffffff',
        panel: 'bg-slate-900',
        panelBorder: 'border-white/5',
        header: 'bg-slate-900 border-blue-900/50',
    },
    light: {
        name: 'Light Orange',
        bg: '#f8fafc', // Slate 50
        grid: '#cbd5e1', // Slate 300
        accent: '#f97316', // Orange 500
        text: '#1e293b', // Slate 800
        panel: 'bg-white',
        panelBorder: 'border-slate-200',
        header: 'bg-white border-orange-200',
    }
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
            <meshBasicMaterial toneMapped={false} />
        </instancedMesh>
    );
};

// Procedural Robot Dog (Unitree Go1 Style)
const RobotDog = ({ color = "#cbd5e1" }) => {
    // A simplified dog: Body + Head + 4 Legs
    return (
        <group>
            {/* Label */}
            <Html position={[0, 0, 0.8]} center distanceFactor={10} style={{ pointerEvents: 'none' }}>
                <div className="bg-black/60 text-white text-base font-bold px-4 py-1 rounded-full border border-blue-400/50 whitespace-nowrap font-mono backdrop-blur-md shadow-[0_0_10px_rgba(59,130,246,0.5)]">
                    Robo Dog
                </div>
            </Html>

            {/* Body */}
            <mesh position={[0, 0, 0]}>
                <boxGeometry args={[0.6, 0.25, 0.2]} />
                <meshStandardMaterial color="#f8fafc" /> {/* Slate 50 */}
            </mesh>
            {/* Head */}
            <mesh position={[0.35, 0, 0.1]}>
                <boxGeometry args={[0.2, 0.15, 0.15]} />
                <meshStandardMaterial color="#e2e8f0" /> {/* Slate 200 */}
            </mesh>
            {/* Legs (Front L/R, Back L/R) */}
            <mesh position={[0.25, 0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#cbd5e1" /> {/* Slate 300 */}
            </mesh>
            <mesh position={[0.25, -0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#cbd5e1" />
            </mesh>
            <mesh position={[-0.25, 0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#cbd5e1" />
            </mesh>
            <mesh position={[-0.25, -0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#cbd5e1" />
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
                <Line points={linePoints} color="#fbbf24" lineWidth={6} /> // Yellow Path
            )}
        </group>
    );
}

const WaypointMarker = ({ wp, onClick, theme }) => {
    return (
        <group position={[wp.x, wp.y, wp.z]}>
            <mesh onClick={(e) => { e.stopPropagation(); onClick(wp); }}>
                <sphereGeometry args={[0.2]} />
                <meshStandardMaterial color={theme.accent} emissive={theme.accent} emissiveIntensity={0.5} />
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
        <div className="w-64 h-48 bg-black/80 flex items-center justify-center text-slate-500 text-xs font-mono border border-slate-700 rounded-lg backdrop-blur-sm">
            NO CAMERA FEED
        </div>
    );

    return (
        <div className="relative border border-blue-500/50 rounded-lg overflow-hidden shadow-2xl bg-black w-64 md:w-80 transition-all duration-300 hover:scale-105">
            <img src={imageSrc} alt="Robot Camera" className="w-full h-auto object-cover" />
            <div className="absolute top-2 right-2 flex gap-1">
                <div className="w-2 h-2 bg-red-500 rounded-full animate-pulse shadow-[0_0_8px_red]"></div>
            </div>
            <div className="absolute bottom-2 left-2 text-[10px] text-blue-200 bg-black/60 px-2 py-0.5 rounded font-mono backdrop-blur-sm border border-white/10">
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

const SidebarCategory = ({ title, icon: Icon, children, defaultOpen = false, theme }) => {
    const [isOpen, setIsOpen] = useState(defaultOpen);
    const isDark = theme.name.includes('Dark');

    return (
        <div className={`border-b ${isDark ? 'border-white/5' : 'border-slate-200'}`}>
            <button
                onClick={() => setIsOpen(!isOpen)}
                className={`w-full flex items-center justify-between p-4 hover:${isDark ? 'bg-white/5' : 'bg-slate-50'} transition-colors`}
            >
                <div className="flex items-center gap-3">
                    {Icon && <Icon size={16} className={isDark ? "text-blue-400" : "text-orange-500"} />}
                    <span className={`text-xs font-bold uppercase tracking-wider ${isDark ? 'text-slate-300' : 'text-slate-700'}`}>{title}</span>
                </div>
                {isOpen ? <ChevronDown size={14} className="opacity-50" /> : <ChevronRight size={14} className="opacity-50" />}
            </button>
            {isOpen && (
                <div className="p-4 pt-0 animate-in slide-in-from-top-2 duration-200">
                    {children}
                </div>
            )}
        </div>
    );
};

const Viewer3D = ({ onBack }) => {
    const [selectedPoint, setSelectedPoint] = useState(null);
    const [waypoints, setWaypoints] = useState([]);
    const [wpName, setWpName] = useState("");
    const [currentTheme, setCurrentTheme] = useState('dark');

    const controlsRef = useRef();
    const prevViewRef = useRef(null);

    const theme = THEMES[currentTheme];
    const isDark = currentTheme === 'dark';

    useEffect(() => {
        const fetchWps = () => {
            fetch('http://localhost:8000/waypoints').then(res => res.json()).then(setWaypoints).catch(e => console.error(e));
        }
        fetchWps();
        const interval = setInterval(fetchWps, 2000);
        return () => clearInterval(interval);
    }, []);

    const toggleTopView = () => {
        if (!controlsRef.current) return;
        const controls = controlsRef.current;
        const camera = controls.object;

        if (prevViewRef.current) {
            // Restore
            const { position, target } = prevViewRef.current;
            camera.position.copy(position);
            controls.target.copy(target);
            prevViewRef.current = null;
        } else {
            // Save
            prevViewRef.current = {
                position: camera.position.clone(),
                target: controls.target.clone()
            };
            // Top View
            const target = controls.target;
            camera.position.set(target.x, target.y + 15, target.z + 0.1);
            controls.target.set(target.x, target.y, target.z);
        }
        controls.update();
    };


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
        <div className={`w-full h-screen flex flex-col font-sans overflow-hidden transition-colors duration-300 ${isDark ? 'text-slate-100 bg-slate-900' : 'text-slate-800 bg-slate-50'}`}>
            {/* Header */}
            <header className={`${theme.header} h-14 flex items-center justify-between px-6 shadow-xl z-20 shrink-0 border-b transition-colors duration-300`}>
                <div className="flex items-center gap-4">
                    <button onClick={onBack} className={`p-1 transition-colors ${isDark ? 'hover:text-blue-400' : 'hover:text-orange-500'}`}>
                        <Layout size={20} />
                    </button>
                    <div className={`text-xl font-bold tracking-wider flex items-center gap-2 ${isDark ? 'text-white' : 'text-slate-800'}`}>
                        <div className={`w-6 h-6 rounded flex items-center justify-center text-[10px] font-black shadow-[0_0_10px_rgba(59,130,246,0.6)] ${isDark ? 'bg-gradient-to-tr from-blue-600 to-cyan-400 text-white' : 'bg-gradient-to-tr from-orange-500 to-amber-400 text-white'}`}>RD</div>
                        Robo Dog<span className={`font-thin ${isDark ? 'text-blue-500' : 'text-orange-500'}`}>OS</span>
                    </div>
                </div>

                <div className="flex items-center gap-6">
                    <div className="flex items-center gap-2 bg-black/10 p-1 rounded-full border border-white/5">
                        <button
                            onClick={() => setCurrentTheme('dark')}
                            className={`p-1.5 rounded-full transition-all ${currentTheme === 'dark' ? 'bg-blue-600 text-white shadow-lg' : 'text-slate-400 hover:text-slate-200'}`}
                        >
                            <Moon size={14} />
                        </button>
                        <button
                            onClick={() => setCurrentTheme('light')}
                            className={`p-1.5 rounded-full transition-all ${currentTheme === 'light' ? 'bg-orange-500 text-white shadow-lg' : 'text-slate-400 hover:text-slate-600'}`}
                        >
                            <Sun size={14} />
                        </button>
                    </div>

                    <div className={`flex items-center gap-4 text-xs font-mono ${isDark ? 'text-blue-300/80' : 'text-slate-500'}`}>
                        <span className="flex items-center gap-2"><div className="w-1.5 h-1.5 bg-green-500 rounded-full shadow-[0_0_5px_lime]"></div> ONLINE</span>
                        <span className="opacity-50">|</span>
                        <span>v1.0.0</span>
                    </div>
                </div>
            </header>

            <div className="flex flex-1 overflow-hidden relative">
                {/* Left Sidebar */}
                <aside className={`w-80 z-10 flex flex-col shadow-2xl transition-colors duration-300 border-r ${theme.panel} ${theme.panelBorder}`}>
                    <div className={`p-4 border-b ${theme.panelBorder}`}>
                        <h2 className={`text-xs font-bold uppercase tracking-[0.2em] ${isDark ? 'text-blue-400' : 'text-orange-600'}`}>Operations</h2>
                    </div>

                    <div className="flex-1 overflow-y-auto">
                        {/* Control Module */}
                        <SidebarCategory title="Teleoperation" icon={Sliders} defaultOpen={true} theme={theme}>
                            {selectedPoint ? (
                                <div className={`border rounded-lg p-4 animate-in fade-in slide-in-from-top-2 ${isDark ? 'bg-blue-900/10 border-blue-500/30' : 'bg-orange-50 border-orange-200'}`}>
                                    <h3 className={`text-[10px] font-bold uppercase mb-3 ${isDark ? 'text-blue-400' : 'text-orange-600'}`}>Target Lock</h3>
                                    <div className={`grid grid-cols-2 gap-2 mb-3 font-mono text-[10px] ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
                                        <div className={`p-1.5 rounded border ${isDark ? 'bg-black/30 border-white/5' : 'bg-white border-slate-200'}`}>X: {selectedPoint.x.toFixed(2)}</div>
                                        <div className={`p-1.5 rounded border ${isDark ? 'bg-black/30 border-white/5' : 'bg-white border-slate-200'}`}>Y: {selectedPoint.y.toFixed(2)}</div>
                                    </div>
                                    <div className="space-y-2">
                                        <button onClick={() => handleNavigate(selectedPoint)} className={`w-full py-2 rounded text-xs font-bold shadow-lg transition-all ${isDark ? 'bg-blue-600 hover:bg-blue-500 text-white shadow-blue-900/20' : 'bg-orange-500 hover:bg-orange-400 text-white shadow-orange-200'}`}>
                                            INITIATE NAV
                                        </button>
                                        <div className="flex gap-2">
                                            <input
                                                value={wpName}
                                                onChange={e => setWpName(e.target.value)}
                                                placeholder="DESIGNATION"
                                                className={`flex-1 border rounded px-2 text-xs focus:outline-none ${isDark ? 'bg-black/30 border-white/10 text-white placeholder-slate-600 focus:border-blue-500' : 'bg-white border-slate-200 text-slate-800 placeholder-slate-400 focus:border-orange-500'}`}
                                            />
                                            <button onClick={handleSaveWaypoint} className={`px-3 rounded text-xs font-bold text-white ${isDark ? 'bg-slate-700 hover:bg-slate-600' : 'bg-slate-500 hover:bg-slate-400'}`}>SV</button>
                                        </div>
                                        <button onClick={() => setSelectedPoint(null)} className="w-full text-[10px] text-slate-500 hover:text-slate-400 mt-1">CANCEL</button>
                                    </div>
                                </div>
                            ) : (
                                <div className={`text-xs text-center p-4 border border-dashed rounded opacity-50 ${isDark ? 'border-slate-700 text-slate-500' : 'border-slate-300 text-slate-400'}`}>
                                    Select point on grid to navigate
                                </div>
                            )}
                        </SidebarCategory>

                        {/* Saved Locations */}
                        <SidebarCategory title="Saved Locations" icon={MapPin} defaultOpen={true} theme={theme}>
                            <div className={`border rounded-lg overflow-hidden ${isDark ? 'bg-slate-800/50 border-white/5' : 'bg-white border-slate-200'}`}>
                                {waypoints.length === 0 ? (
                                    <div className="p-4 text-center text-xs text-slate-500 font-mono">NO DATA</div>
                                ) : (
                                    <ul className={`divide-y ${isDark ? 'divide-white/5' : 'divide-slate-100'}`}>
                                        {waypoints.map((wp, i) => (
                                            <li key={i} className={`flex items-center justify-between p-3 transition-colors group ${isDark ? 'hover:bg-white/5' : 'hover:bg-slate-50'}`}>
                                                <span className={`text-sm font-medium ${isDark ? 'text-slate-300' : 'text-slate-600'}`}>{wp.name}</span>
                                                <div className="flex gap-2 opacity-0 group-hover:opacity-100 transition-opacity">
                                                    <button onClick={() => handleNavigate(wp)} className={`text-xs font-bold ${isDark ? 'text-blue-400 hover:text-blue-300' : 'text-orange-500 hover:text-orange-400'}`}>GO</button>
                                                    <button onClick={() => handleDeleteWaypoint(wp.name)} className="text-xs text-red-500 hover:text-red-400">X</button>
                                                </div>
                                            </li>
                                        ))}
                                    </ul>
                                )}
                            </div>
                        </SidebarCategory>

                        {/* System Status (Placeholder for more categories) */}
                        <SidebarCategory title="System Status" icon={Settings} theme={theme}>
                            <div className="space-y-2">
                                <div className={`flex justify-between items-center text-xs ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
                                    <span>Battery</span>
                                    <span className="text-green-500 font-mono">98%</span>
                                </div>
                                <div className={`h-1.5 w-full rounded-full overflow-hidden ${isDark ? 'bg-slate-800' : 'bg-slate-200'}`}>
                                    <div className="h-full bg-green-500 w-[98%]"></div>
                                </div>
                                <div className={`flex justify-between items-center text-xs pt-2 ${isDark ? 'text-slate-400' : 'text-slate-600'}`}>
                                    <span>Signal</span>
                                    <span className="text-blue-500 font-mono">-42dBm</span>
                                </div>
                            </div>
                        </SidebarCategory>

                    </div>

                    {/* Sidebar Footer */}
                    <div className={`p-4 border-t text-[10px] text-center opacity-40 ${theme.panelBorder} ${isDark ? 'text-slate-500' : 'text-slate-400'}`}>
                        MISSION PLANNER UI
                    </div>
                </aside>

                {/* 3D Viewport */}
                <div className={`flex-1 relative ${theme.bg} transition-colors duration-300`}>
                    <div className="absolute top-4 left-4 z-10 pointer-events-none">
                        <div className={`backdrop-blur border p-2 rounded text-[10px] font-mono shadow-sm ${isDark ? 'bg-black/60 border-white/10 text-blue-300' : 'bg-white/80 border-slate-200 text-slate-600'}`}>
                            LAT: 2s :: VOX_GRID [{VOXEL_SIZE}m]
                        </div>
                    </div>

                    {/* Top View Toggle */}
                    <div className="absolute top-4 right-4 z-20 flex flex-col gap-2">
                        <button
                            onClick={toggleTopView}
                            className={`p-2 rounded-lg backdrop-blur-md border shadow-lg transition-all ${isDark ? 'bg-black/40 border-white/10 text-blue-300 hover:bg-black/60 hover:text-white' : 'bg-white/60 border-slate-200 text-slate-600 hover:bg-white hover:text-slate-900'}`}
                            title="Toggle Top View"
                        >
                            <ArrowUp size={20} />
                        </button>
                    </div>

                    {/* Camera Feed Overlay - Bottom Right */}
                    <div className="absolute bottom-6 right-6 z-20 flex flex-col items-end gap-2">
                        <div className={`text-[10px] font-bold uppercase tracking-wider mb-1 px-2 py-0.5 rounded backdrop-blur-md border ${isDark ? 'text-slate-400 bg-black/40 border-white/5' : 'text-slate-600 bg-white/60 border-slate-200'}`}>
                            Live Feed
                        </div>
                        <CameraFeed />
                    </div>

                    <Canvas camera={{ position: [5, 5, 5], fov: 50 }} dpr={[1, 2]} shadows>
                        <color attach="background" args={[theme.bg]} />
                        <fog attach="fog" args={[theme.bg, 10, 50]} />

                        <ambientLight intensity={0.5} />
                        <spotLight position={[10, 20, 10]} intensity={1} castShadow shadow-mapSize={2048} />
                        <pointLight position={[-10, -10, -10]} intensity={0.5} color={theme.accent} />

                        <gridHelper args={[60, 60, theme.grid, theme.grid]} />
                        <axesHelper args={[2]} />

                        <group rotation={[-Math.PI / 2, 0, 0]}>
                            <VoxelCloud url="ws://localhost:8000/ws/points" />
                            <DataVisualizer url="ws://localhost:8000/ws/tf" />
                            {waypoints.map((wp, i) => <WaypointMarker key={i} wp={wp} onClick={handleNavigate} theme={theme} />)}

                            {selectedPoint && (
                                <mesh position={[selectedPoint.x, selectedPoint.y, selectedPoint.z]}>
                                    <sphereGeometry args={[0.25]} />
                                    <meshBasicMaterial color="#ef4444" transparent opacity={0.6} wireframe />
                                </mesh>
                            )}
                        </group>

                        <InteractionPlane onPointSelected={setSelectedPoint} />
                        <OrbitControls ref={controlsRef} makeDefault minPolarAngle={0} maxPolarAngle={Math.PI / 1.7} />
                    </Canvas>
                </div>
            </div>
        </div >
    );
};

export default Viewer3D;
