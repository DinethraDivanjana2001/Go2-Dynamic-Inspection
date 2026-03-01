import React, { useEffect, useRef, useState, useMemo } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Line, Html } from '@react-three/drei';
import * as THREE from 'three';
import { ChevronDown, ChevronRight, Video, MapPin, Sliders, Settings, Sun, Moon, Layout, ArrowUp, User as UserIcon, Satellite, Flag, CloudSun, Clock, Battery } from 'lucide-react';

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

const VoxelCloud = ({ url, brightness = 1.0 }) => {
    const meshRef = useRef();
    const MAX_INSTANCES = 150000;
    const dummy = useMemo(() => new THREE.Object3D(), []);

    useEffect(() => {
        const token = localStorage.getItem('auth_token');
        const wsUrl = url + (url.includes('?') ? '&' : '?') + `token=${token}`;
        const ws = new WebSocket(wsUrl);
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

    // Apply brightness (lum) logic directly via the material without waiting for a new socket message
    // Use a slightly more aggressive curve for opacity to make it more felt
    const isTransparent = brightness < 1.0;
    const opacityVal = isTransparent ? Math.pow(brightness, 1.5) : 1.0;
    const colorMultiplier = Math.max(1.0, brightness);

    return (
        <instancedMesh ref={meshRef} args={[null, null, MAX_INSTANCES]}>
            <boxGeometry args={[VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE]} />
            <meshBasicMaterial
                toneMapped={false}
                transparent={isTransparent}
                opacity={opacityVal}
                depthWrite={!isTransparent} // Crucial for seeing through many voxels
                color={new THREE.Color(colorMultiplier, colorMultiplier, colorMultiplier)}
            />
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
                <div className="bg-black/80 text-white text-base font-bold px-4 py-1 rounded-full border-[2px] border-blue-400 whitespace-nowrap font-mono shadow-[0_0_15px_rgba(59,130,246,0.8)] flex items-center gap-2">
                    <div className="w-2.5 h-2.5 bg-blue-400 rounded-full animate-pulse shadow-[0_0_8px_rgba(59,130,246,1)]"></div>
                    Robo Dog
                </div>
            </Html>

            {/* Glowing Ground Ring */}
            <mesh position={[0, 0, -0.35]}>
                <ringGeometry args={[0.6, 0.7, 32]} />
                <meshBasicMaterial color="#3b82f6" transparent opacity={0.6} side={THREE.DoubleSide} />
            </mesh>
            <mesh position={[0, 0, -0.35]}>
                <circleGeometry args={[0.6, 32]} />
                <meshBasicMaterial color="#3b82f6" transparent opacity={0.15} side={THREE.DoubleSide} />
            </mesh>

            {/* Body */}
            <mesh position={[0, 0, 0]}>
                <boxGeometry args={[0.6, 0.25, 0.2]} />
                <meshStandardMaterial color="#ffffff" emissive="#3b82f6" emissiveIntensity={0.2} />
            </mesh>
            {/* Head */}
            <mesh position={[0.35, 0, 0.1]}>
                <boxGeometry args={[0.2, 0.15, 0.15]} />
                <meshStandardMaterial color="#ffffff" emissive="#3b82f6" emissiveIntensity={0.4} />
            </mesh>

            {/* Legs (Front L/R, Back L/R) */}
            <mesh position={[0.25, 0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#cbd5e1" emissive="#3b82f6" emissiveIntensity={0.1} />
            </mesh>
            <mesh position={[0.25, -0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#cbd5e1" emissive="#3b82f6" emissiveIntensity={0.1} />
            </mesh>
            <mesh position={[-0.25, 0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#cbd5e1" emissive="#3b82f6" emissiveIntensity={0.1} />
            </mesh>
            <mesh position={[-0.25, -0.15, -0.2]}>
                <boxGeometry args={[0.08, 0.08, 0.4]} />
                <meshStandardMaterial color="#cbd5e1" emissive="#3b82f6" emissiveIntensity={0.1} />
            </mesh>
        </group>
    )
}

// Calculate Euclidean Distance of path
const calculatePathLength = (path) => {
    if (!path || path.length < 2) return 0;
    let dist = 0;
    for (let i = 0; i < path.length - 1; i++) {
        const dx = path[i + 1].x - path[i].x;
        const dy = path[i + 1].y - path[i].y;
        const dz = path[i + 1].z - path[i].z;
        dist += Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    return dist;
};

const DataVisualizer = ({ url, onPathUpdate }) => {
    const [tfs, setTfs] = useState({});
    const [path, setPath] = useState([]);

    // Refs for speed calculation
    const lastPosRef = useRef(null);
    const lastTimeRef = useRef(null);

    useEffect(() => {
        const token = localStorage.getItem('auth_token');
        const wsUrl = url + (url.includes('?') ? '&' : '?') + `token=${token}`;
        const ws = new WebSocket(wsUrl);
        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);

                // --- Speed Calculation ---
                if (data.tfs && data.tfs["base_link"]) {
                    const currentPos = data.tfs["base_link"].translation;
                    const now = performance.now();

                    if (lastPosRef.current && lastTimeRef.current) {
                        const dt = (now - lastTimeRef.current) / 1000.0; // seconds
                        if (dt > 0.05) { // Throttle calculations slightly to avoid noise
                            const dx = currentPos.x - lastPosRef.current.x;
                            const dy = currentPos.y - lastPosRef.current.y;
                            const dz = currentPos.z - lastPosRef.current.z;

                            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
                            const speed = dist / dt;

                            // Emit global event with smoothed speed and mqtt rate
                            window.dispatchEvent(new CustomEvent('dashboardMetricsUpdate', {
                                detail: { speed, mqttRate: data.mqtt_rate || 0 }
                            }));

                            lastPosRef.current = currentPos;
                            lastTimeRef.current = now;
                        }
                    } else {
                        lastPosRef.current = currentPos;
                        lastTimeRef.current = now;
                    }
                }

                if (data.tfs) setTfs(data.tfs);
                if (data.path) {
                    setPath(data.path);
                    if (onPathUpdate) onPathUpdate(data.path);
                }
            } catch (e) { }
        };
        return () => ws.close();
    }, [url, onPathUpdate]);

    const linePoints = useMemo(() => path.map(p => [p.x, p.y, p.z]), [path]);

    return (
        <group>
            {Object.entries(tfs).map(([childId, tf]) => {
                const p = [tf.translation.x, tf.translation.y, tf.translation.z];
                const q = new THREE.Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);

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
                <Line points={linePoints} color="#fbbf24" lineWidth={6} />
            )}
        </group>
    );
}

const WaypointMarker = ({ wp, index = 0, isCompleted = false, onClick, theme }) => {
    return (
        <group position={[wp.x, wp.y, wp.z]}>
            <Html position={[0, 0, 0.4]} center distanceFactor={24} style={{ pointerEvents: 'none' }} className="group">
                <div
                    onClick={(e) => { e.stopPropagation(); onClick(wp); }}
                    className="relative flex flex-col items-center justify-center cursor-pointer transition-transform hover:scale-110 pointer-events-auto"
                >
                    {/* Map Pin Head */}
                    <div className={`min-w-[48px] px-2 h-12 rounded-full shadow-[0_4px_12px_rgba(0,0,0,0.6)] flex items-center justify-center text-white font-bold border-[3px] backdrop-blur-md ${isCompleted ? 'bg-red-500/90 border-red-200' : 'bg-blue-500/90 border-blue-200'} gap-1.5`}>
                        {isCompleted ? (
                            <svg className="w-5 h-5 text-white drop-shadow-md shrink-0" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={3} d="M5 13l4 4L19 7" /></svg>
                        ) : (
                            <span className="text-xl drop-shadow-md shrink-0">{index + 1}</span>
                        )}
                        <span className="text-xs uppercase tracking-wider opacity-90 truncate max-w-[80px] drop-shadow-md whitespace-nowrap">{wp.name}</span>
                    </div>
                    {/* Map Pin Tip */}
                    <div className={`w-0 h-0 border-l-[8px] border-r-[8px] border-t-[12px] border-l-transparent border-r-transparent ${isCompleted ? 'border-t-red-500/90' : 'border-t-blue-500/90'} -mt-0.5 drop-shadow-md`}></div>
                </div>
            </Html>
        </group>
    )
}

const CameraFeed = () => {
    const [imageSrc, setImageSrc] = useState(null);

    useEffect(() => {
        const token = localStorage.getItem('auth_token');
        const HOST = window.location.hostname;
        const ws = new WebSocket(`ws://${HOST}:8000/ws/video?token=${token}`);
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
    const pointerDownPos = useRef(null);

    return (
        <mesh
            rotation={[-Math.PI / 2, 0, 0]}
            position={[0, 0, 0]}
            onPointerDown={(e) => {
                pointerDownPos.current = { x: e.clientX, y: e.clientY };
            }}
            onPointerUp={(e) => {
                if (!pointerDownPos.current) return;
                const dist = Math.sqrt(
                    Math.pow(e.clientX - pointerDownPos.current.x, 2) +
                    Math.pow(e.clientY - pointerDownPos.current.y, 2)
                );
                pointerDownPos.current = null;

                if (dist < 5) {
                    e.stopPropagation();
                    const worldP = e.point;
                    onPointSelected({ x: worldP.x, y: -worldP.z, z: worldP.y });
                }
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

const DashboardMetrics = ({ theme }) => {
    const [time, setTime] = useState(new Date());
    const [speed, setSpeed] = useState(0);
    const [mqttRate, setMqttRate] = useState(0);
    const [weather, setWeather] = useState({ temp: "--" });
    const isDark = theme.name.includes('Dark');

    // Clock
    useEffect(() => {
        const timer = setInterval(() => setTime(new Date()), 60000); // 1 min update
        return () => clearInterval(timer);
    }, []);

    // Weather Fetch (IP based)
    useEffect(() => {
        const fetchWeather = async () => {
            try {
                // Get approx lat/lon from IP
                const locRes = await fetch('https://ipapi.co/json/');
                const locData = await locRes.json();

                if (locData.latitude && locData.longitude) {
                    // Fetch temp from open-meteo
                    const wRes = await fetch(`https://api.open-meteo.com/v1/forecast?latitude=${locData.latitude}&longitude=${locData.longitude}&current=temperature_2m&temperature_unit=fahrenheit`);
                    const wData = await wRes.json();

                    if (wData.current && wData.current.temperature_2m) {
                        setWeather({ temp: Math.round(wData.current.temperature_2m) });
                    }
                }
            } catch (error) {
                console.error("Failed to fetch weather", error);
            }
        };

        fetchWeather();
        const weatherTimer = setInterval(fetchWeather, 300000); // Update every 5 mins
        return () => clearInterval(weatherTimer);
    }, []);

    // Websocket Telemetry Listener
    useEffect(() => {
        const handleUpdate = (e) => {
            setSpeed(e.detail.speed);
            setMqttRate(e.detail.mqttRate);
        };

        window.addEventListener('dashboardMetricsUpdate', handleUpdate);
        return () => window.removeEventListener('dashboardMetricsUpdate', handleUpdate);
    }, []);

    const formatTime = (date) => {
        return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
    };

    return (
        <div className={`p-4 border-t flex flex-col gap-4 font-mono ${theme.panelBorder} ${isDark ? 'text-slate-300' : 'text-slate-600'}`}>
            <div className="flex items-center gap-4">
                <Satellite size={20} className={isDark ? "text-slate-400" : "text-slate-500"} />
                <span className="text-xl font-medium tracking-widest">{mqttRate}</span>
            </div>

            <div className="flex items-center gap-4">
                <Flag size={20} className={isDark ? "text-slate-400" : "text-slate-500"} />
                <div className="flex items-baseline gap-1.5">
                    <span className="text-xl font-medium tracking-widest">{speed.toFixed(1)}</span>
                    <span className="text-sm">m/s</span>
                </div>
            </div>

            <div className="flex items-center gap-4">
                <CloudSun size={20} className={isDark ? "text-slate-400" : "text-slate-500"} />
                <span className="text-xl font-medium tracking-widest">{weather.temp}°</span>
            </div>

            <div className="flex items-center gap-4 pt-2 mt-2 border-t border-dashed border-slate-500/30">
                <Clock size={16} className={isDark ? "text-slate-500" : "text-slate-400"} />
                <span className="text-sm font-bold tracking-widest">{formatTime(time)}</span>
            </div>
        </div>
    );
};


const Viewer3D = ({ onBack, onLogout }) => {
    const [selectedPoint, setSelectedPoint] = useState(null);
    const [username, setUsername] = useState("");
    const [displayName, setDisplayName] = useState("");
    const [isEditingName, setIsEditingName] = useState(false);
    const [newName, setNewName] = useState("");
    const [pointCloudBrightness, setPointCloudBrightness] = useState(1.0);

    useEffect(() => {
        const token = localStorage.getItem('auth_token');
        if (token) {
            try {
                const payload = JSON.parse(atob(token.split('.')[1]));
                if (payload.sub) setUsername(payload.sub);
            } catch (e) {
                console.error("Failed to parse token");
            }
        }
    }, []);
    const [waypoints, setWaypoints] = useState([]);
    const [wpName, setWpName] = useState("");
    const [currentTheme, setCurrentTheme] = useState('dark');
    const [pathLength, setPathLength] = useState(0.0);
    const [profilePic, setProfilePic] = useState(null);

    const controlsRef = useRef();
    const prevViewRef = useRef(null);

    const theme = THEMES[currentTheme];
    const isDark = currentTheme === 'dark';

    useEffect(() => {
        const fetchWps = () => {
            const token = localStorage.getItem('auth_token');
            const HOST = window.location.hostname;

            // Fetch Waypoints
            fetch(`http://${HOST}:8000/waypoints`, {
                headers: { 'Authorization': `Bearer ${token}` }
            }).then(res => {
                if (res.ok) return res.json();
                throw new Error("Unauthorized");
            }).then(data => setWaypoints(data || [])).catch(e => console.error(e));

            // Fetch Profile Picture and DisplayName
            fetch(`http://${HOST}:8000/profile`, {
                headers: { 'Authorization': `Bearer ${token}` }
            }).then(res => {
                if (res.ok) return res.json();
                return null;
            }).then(data => {
                if (data) {
                    if (data.profile_pic) setProfilePic(data.profile_pic);
                    if (data.display_name) setDisplayName(data.display_name);
                }
            }).catch(e => console.error(e));
        }
        fetchWps();
        const interval = setInterval(fetchWps, 2000);
        return () => clearInterval(interval);
    }, []);

    const handleUpdateName = () => {
        if (!newName.trim()) {
            setIsEditingName(false);
            return;
        }
        const token = localStorage.getItem('auth_token');
        const HOST = window.location.hostname;
        fetch(`http://${HOST}:8000/profile`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${token}`
            },
            body: JSON.stringify({ display_name: newName })
        }).then(res => {
            if (res.ok) {
                setDisplayName(newName);
            }
        }).catch(console.error)
            .finally(() => setIsEditingName(false));
    };

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
        const token = localStorage.getItem('auth_token');
        const HOST = window.location.hostname;
        fetch(`http://${HOST}:8000/navigate`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${token}`
            },
            body: JSON.stringify({ x: point.x, y: point.y, z: point.z })
        }).then(() => setSelectedPoint(null));
    };

    const handleSaveWaypoint = () => {
        if (!selectedPoint || !wpName) return;
        const token = localStorage.getItem('auth_token');
        const HOST = window.location.hostname;
        fetch(`http://${HOST}:8000/waypoints`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${token}`
            },
            body: JSON.stringify({ name: wpName, x: selectedPoint.x, y: selectedPoint.y, z: selectedPoint.z })
        }).then(res => res.json()).then(data => {
            setWaypoints(data.waypoints || []);
            setWpName("");
            setSelectedPoint(null);
        });
    };

    const handleDeleteWaypoint = (name) => {
        const token = localStorage.getItem('auth_token');
        const HOST = window.location.hostname;
        fetch(`http://${HOST}:8000/waypoints/${name}`, {
            method: 'DELETE',
            headers: { 'Authorization': `Bearer ${token}` }
        }).then(res => res.json()).then(data => setWaypoints(data.waypoints || []));
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

                <div className={`hidden md:flex items-center gap-4 text-xs font-mono border-x px-6 ${isDark ? 'border-white/10 text-slate-400' : 'border-slate-200 text-slate-500'}`}>
                    <div className="flex flex-col items-center">
                        <span className="text-[10px] uppercase tracking-wider opacity-60">Path Distance</span>
                        <span className={`text-lg font-bold ${isDark ? 'text-blue-400' : 'text-orange-500'}`}>{pathLength.toFixed(2)}m</span>
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
                        <span className="flex items-center gap-1.5">
                            <Battery size={14} className={isDark ? "text-blue-400" : "text-orange-500"} />
                            <span className="font-bold">85%</span>
                        </span>
                        <span className="opacity-50">|</span>
                        <span>v1.0.0</span>
                    </div>
                </div>
            </header>

            <div className="flex flex-1 overflow-hidden relative">
                {/* Left Sidebar */}
                <aside className={`w-80 z-10 flex flex-col shadow-2xl transition-colors duration-300 border-r ${theme.panel} ${theme.panelBorder}`}>

                    {/* Top User Profile Block */}
                    <div className={`p-6 border-b relative flex flex-col items-center justify-center ${theme.panelBorder}`}>
                        <button
                            className={`absolute top-4 right-4 p-1.5 rounded transition-all ${isDark ? 'text-slate-400 hover:text-white hover:bg-white/10' : 'text-slate-500 hover:text-slate-900 hover:bg-slate-100'}`}
                            onClick={() => {
                                setNewName(displayName || username);
                                setIsEditingName(!isEditingName);
                            }}
                        >
                            <Settings size={18} />
                        </button>

                        <div className="relative group cursor-pointer mb-3" onClick={() => document.getElementById('profileUpload').click()}>
                            <div className={`w-20 h-20 rounded-full overflow-hidden border-2 flex items-center justify-center shrink-0 shadow-lg ${isDark ? 'border-slate-600 bg-slate-800' : 'border-slate-300 bg-slate-200'}`}>
                                {profilePic ? (
                                    <img src={profilePic} alt="Profile" className="w-full h-full object-cover" />
                                ) : (
                                    <UserIcon size={32} className={isDark ? "text-slate-600" : "text-slate-400"} />
                                )}
                            </div>
                            <div className="absolute inset-0 bg-black/50 rounded-full opacity-0 group-hover:opacity-100 flex items-center justify-center transition-opacity">
                                <span className="text-[10px] font-bold text-white uppercase text-center leading-tight">Change<br />Pic</span>
                            </div>
                            <input
                                type="file"
                                id="profileUpload"
                                accept="image/*"
                                className="hidden"
                                onChange={(e) => {
                                    const file = e.target.files?.[0];
                                    if (!file) return;

                                    const reader = new FileReader();
                                    reader.onload = (event) => {
                                        const img = new Image();
                                        img.onload = () => {
                                            const canvas = document.createElement('canvas');
                                            const MAX_SIZE = 150;
                                            let width = img.width;
                                            let height = img.height;

                                            if (width > height) {
                                                if (width > MAX_SIZE) {
                                                    height *= MAX_SIZE / width;
                                                    width = MAX_SIZE;
                                                }
                                            } else {
                                                if (height > MAX_SIZE) {
                                                    width *= MAX_SIZE / height;
                                                    height = MAX_SIZE;
                                                }
                                            }

                                            canvas.width = width;
                                            canvas.height = height;
                                            const ctx = canvas.getContext('2d');
                                            ctx.drawImage(img, 0, 0, width, height);

                                            const dataUrl = canvas.toDataURL('image/jpeg', 0.8);
                                            setProfilePic(dataUrl);

                                            const token = localStorage.getItem('auth_token');
                                            const HOST = window.location.hostname;
                                            fetch(`http://${HOST}:8000/profile`, {
                                                method: 'POST',
                                                headers: {
                                                    'Content-Type': 'application/json',
                                                    'Authorization': `Bearer ${token}`
                                                },
                                                body: JSON.stringify({ profile_pic: dataUrl })
                                            }).catch(console.error);
                                        };
                                        img.src = event.target.result;
                                    };
                                    reader.readAsDataURL(file);
                                    e.target.value = null;
                                }}
                            />
                        </div>

                        {isEditingName ? (
                            <div className="flex flex-col items-center w-full px-4 mb-2">
                                <input
                                    type="text"
                                    value={newName}
                                    onChange={(e) => setNewName(e.target.value)}
                                    className={`w-full text-center text-sm font-bold uppercase tracking-wider bg-transparent border-b outline-none pb-1 ${isDark ? 'text-white border-blue-500' : 'text-slate-800 border-orange-500'}`}
                                    autoFocus
                                    onKeyDown={(e) => {
                                        if (e.key === 'Enter') handleUpdateName();
                                        if (e.key === 'Escape') setIsEditingName(false);
                                    }}
                                />
                                <div className="flex gap-2 mt-2">
                                    <button onClick={handleUpdateName} className={`text-[10px] px-2 py-1 rounded font-bold text-white ${isDark ? 'bg-blue-600 hover:bg-blue-500' : 'bg-orange-500 hover:bg-orange-400'}`}>SAVE</button>
                                    <button onClick={() => setIsEditingName(false)} className={`text-[10px] px-2 py-1 rounded font-bold ${isDark ? 'bg-slate-700 hover:bg-slate-600 text-slate-300' : 'bg-slate-200 hover:bg-slate-300 text-slate-700'}`}>CANCEL</button>
                                </div>
                            </div>
                        ) : (
                            <h2 className={`text-sm font-bold uppercase tracking-wider mb-2 ${isDark ? 'text-white' : 'text-slate-800'}`}>
                                {displayName || username || 'GUEST USER'}
                            </h2>
                        )}

                        <div className={`flex items-center gap-2 px-3 py-1 rounded-full border ${isDark ? 'bg-black/40 border-white/5' : 'bg-slate-100 border-slate-200'}`}>
                            <span className={`text-[10px] font-bold ${isDark ? 'text-slate-400' : 'text-slate-500'}`}>CONNECTED</span>
                            <div className="flex gap-1">
                                <div className="w-2 h-2 rounded-full bg-blue-500 shadow-[0_0_5px_rgba(59,130,246,0.6)]"></div>
                                <div className="w-2 h-2 rounded-full bg-red-500 shadow-[0_0_5px_rgba(239,68,68,0.6)]"></div>
                            </div>
                        </div>
                    </div>

                    <div className="flex-1 overflow-y-auto">
                        {/* Control Module */}
                        <SidebarCategory title="Teleoperation" icon={Sliders} defaultOpen={true} theme={theme}>
                            <div className="p-4 text-xs text-slate-500 text-center italic">
                                Use the Map to Navigate
                            </div>
                        </SidebarCategory>

                        {/* Saved Locations */}
                        <SidebarCategory title="Saved Locations" icon={MapPin} defaultOpen={true} theme={theme}>
                            <div className={`border rounded-lg overflow-hidden ${isDark ? 'bg-slate-800/50 border-white/5' : 'bg-white border-slate-200'}`}>
                                {(!waypoints || waypoints.length === 0) ? (
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

                        {/* System Control (was User Settings) */}
                        <SidebarCategory title="System Control" icon={Settings} theme={theme} defaultOpen={true}>
                            <div className="flex flex-col gap-4">
                                <button
                                    onClick={onLogout}
                                    className="w-full py-2 bg-red-500/90 hover:bg-red-500 text-white text-xs font-bold rounded shadow transition-colors"
                                >
                                    LOGOUT
                                </button>
                            </div>
                        </SidebarCategory>

                        {/* System Status */}
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
                    <DashboardMetrics theme={theme} />
                </aside>

                {/* 3D Viewport */}
                <div className={`flex-1 relative ${theme.bg} transition-colors duration-300`}>
                    <div className="absolute top-4 left-4 z-10 pointer-events-none">
                        <div className={`backdrop-blur border p-2 rounded text-[10px] font-mono shadow-sm ${isDark ? 'bg-black/60 border-white/10 text-blue-300' : 'bg-white/80 border-slate-200 text-slate-600'}`}>
                            LAT: 2s :: VOX_GRID [{VOXEL_SIZE}m]
                        </div>
                    </div>

                    {/* Top View Toggle & Brightness Control */}
                    <div className="absolute top-4 right-4 z-20 flex flex-col gap-2 pointer-events-none">
                        <div className={`p-3 rounded-lg backdrop-blur-md border shadow-lg flex flex-col gap-2 pointer-events-auto ${isDark ? 'bg-black/60 border-white/10' : 'bg-white/80 border-slate-200'}`}>

                            <div className="flex justify-between items-center gap-4">
                                <span className={`text-[10px] font-bold uppercase tracking-widest ${isDark ? 'text-slate-400' : 'text-slate-500'}`}>Pointcloud Lum</span>
                                <span className={`font-mono text-[10px] ${isDark ? 'text-blue-400' : 'text-orange-500'}`}>{(pointCloudBrightness * 100).toFixed(0)}%</span>
                            </div>
                            <input
                                type="range"
                                min="0.1"
                                max="3.0"
                                step="0.1"
                                value={pointCloudBrightness}
                                onChange={(e) => setPointCloudBrightness(parseFloat(e.target.value))}
                                className={`w-32 h-1.5 rounded-full appearance-none outline-none cursor-pointer ${isDark ? 'bg-slate-700/50' : 'bg-slate-300'} [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:w-3 [&::-webkit-slider-thumb]:h-3 [&::-webkit-slider-thumb]:rounded-full ${isDark ? '[&::-webkit-slider-thumb]:bg-blue-400' : '[&::-webkit-slider-thumb]:bg-orange-500'}`}
                            />

                            <div className="w-full h-px bg-slate-500/20 my-1"></div>

                            <button
                                onClick={toggleTopView}
                                className={`p-1.5 rounded flex items-center gap-2 justify-center transition-all ${isDark ? 'hover:bg-white/10 text-slate-300 hover:text-white' : 'hover:bg-slate-200 text-slate-600 hover:text-slate-900'}`}
                                title="Toggle Top View"
                            >
                                <ArrowUp size={14} />
                                <span className="text-[10px] font-bold uppercase tracking-wider">Top View</span>
                            </button>
                        </div>
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
                        <fog attach="fog" args={[theme.bg, 50, 300]} />

                        <ambientLight intensity={0.5} />
                        <spotLight position={[10, 20, 10]} intensity={1} castShadow shadow-mapSize={2048} />
                        <pointLight position={[-10, -10, -10]} intensity={0.5} color={theme.accent} />

                        <gridHelper args={[60, 60, theme.grid, theme.grid]} />
                        <axesHelper args={[2]} />

                        <group rotation={[-Math.PI / 2, 0, 0]}>
                            <VoxelCloud url={`ws://${window.location.hostname}:8000/ws/points`} brightness={pointCloudBrightness} />
                            <DataVisualizer
                                url={`ws://${window.location.hostname}:8000/ws/tf`}
                                onPathUpdate={(path) => setPathLength(calculatePathLength(path))}
                            />
                            {waypoints.map((wp, i) => <WaypointMarker key={i} index={i} isCompleted={i < 2 && waypoints.length > 2} wp={wp} onClick={handleNavigate} theme={theme} />)}

                            {/* Route Line Connecting Waypoints */}
                            {waypoints.length > 1 && (
                                <Line
                                    points={waypoints.map(wp => [wp.x, wp.y, wp.z])}
                                    color="#ffffff"
                                    lineWidth={4}
                                    transparent
                                    opacity={0.8}
                                />
                            )}

                            {selectedPoint && (
                                <group position={[selectedPoint.x, selectedPoint.y, selectedPoint.z]}>
                                    <mesh>
                                        <sphereGeometry args={[0.25]} />
                                        <meshBasicMaterial color="#ef4444" transparent opacity={0.6} wireframe />
                                    </mesh>

                                    {/* Navigation Popup */}
                                    <Html position={[0, 0, 0.5]} center style={{ pointerEvents: 'auto' }}>
                                        <div
                                            onPointerDown={e => e.stopPropagation()}
                                            onPointerUp={e => e.stopPropagation()}
                                            onClick={e => e.stopPropagation()}
                                            className={`w-48 p-2 rounded-lg border shadow-xl backdrop-blur-md animate-in zoom-in-50 duration-200 ${isDark ? 'bg-slate-900/90 border-blue-500/50 text-white' : 'bg-white/90 border-orange-500/50 text-slate-800'}`}>
                                            <div className="text-[10px] font-mono opacity-50 mb-1 border-b pb-1 border-white/10 uppercase tracking-wider text-center">Location Target</div>
                                            <div className="flex justify-between text-[10px] font-mono mb-2 px-2">
                                                <span>X: {selectedPoint.x.toFixed(1)}</span>
                                                <span>Y: {selectedPoint.y.toFixed(1)}</span>
                                            </div>
                                            <button
                                                onClick={() => handleNavigate(selectedPoint)}
                                                className={`w-full py-1.5 rounded text-xs font-bold mb-2 shadow-lg hover:scale-105 active:scale-95 transition-all ${isDark ? 'bg-blue-600 hover:bg-blue-500' : 'bg-orange-500 hover:bg-orange-400'} text-white`}
                                            >
                                                Navigate Here
                                            </button>
                                            <div className="flex gap-1">
                                                <input
                                                    value={wpName}
                                                    onChange={e => setWpName(e.target.value)}
                                                    onPointerDown={e => e.stopPropagation()}
                                                    onClick={e => e.stopPropagation()}
                                                    onKeyDown={e => e.stopPropagation()}
                                                    placeholder="Name..."
                                                    className={`flex-1 text-[10px] px-1 rounded border bg-transparent focus:outline-none ${isDark ? 'border-white/20 focus:border-blue-500' : 'border-slate-300 focus:border-orange-500'}`}
                                                />
                                                <button onClick={handleSaveWaypoint} className={`px-2 py-0.5 rounded text-[10px] font-bold ${isDark ? 'bg-slate-700 hover:bg-slate-600' : 'bg-slate-200 hover:bg-slate-300'}`}>
                                                    Save
                                                </button>
                                            </div>
                                            <button
                                                onClick={() => setSelectedPoint(null)}
                                                className="absolute -top-2 -right-2 w-5 h-5 bg-red-500 hover:bg-red-400 rounded-full text-white text-[10px] font-bold shadow-md flex items-center justify-center"
                                            >
                                                ✕
                                            </button>
                                        </div>
                                    </Html>
                                </group>
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
