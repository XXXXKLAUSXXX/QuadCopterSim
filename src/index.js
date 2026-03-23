//------------------------------------------------------------------------------
//  A basic quadcopter simulator built using Three.js and C (WASM).
//  This module implements user input logic, rendering and communication with
//  the WASM module.
//
//  Author: Andrea Pavan
//  License: MIT
//------------------------------------------------------------------------------
import * as THREE from "./threejs/three.module.min.js";
import { OrbitControls } from "./threejs/addons/controls/OrbitControls.js";
//import {Sky} from "./threejs/addons/objects/Sky.js";
import Stats from "./threejs/addons/libs/stats.module.js";
import { STLLoader } from "./threejs/addons/loaders/STLLoader.js";


//setup scene, camera and renderer
const scene = new THREE.Scene();
//scene.background = new THREE.Color(0xf2f2f2);
scene.background = new THREE.Color(0x5e5d5d);
scene.fog = new THREE.Fog(scene.background, 2.0, 20);
const camera = new THREE.PerspectiveCamera(70, window.innerWidth / window.innerHeight, 0.1, 100);
camera.position.set(0, 0.6, 0.7);
scene.add(camera);
const renderer = new THREE.WebGLRenderer({
    antialias: true
});
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
document.body.appendChild(renderer.domElement);
let paused = true;

//fps statistics
const stats = new Stats();
document.body.appendChild(stats.domElement);

//orbit controls
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;
controls.enablePan = false;
controls.screenSpacePanning = false;
controls.maxPolarAngle = Math.PI / 2 - 0.1;
controls.listenToKeyEvents(window);

//draw sky
/*const sky = new Sky();
sky.scale.setScalar(1e9);
const skySunPosition = new THREE.Vector3().setFromSphericalCoords(
    1,                                  //radius (not relevant)
    THREE.MathUtils.degToRad(80),       //elevation angle (90° = horizon)
    THREE.MathUtils.degToRad(90)        //azimuth angle (90° = east)
);
sky.material.uniforms.sunPosition.value = skySunPosition;
scene.add(sky);*/

//draw floor
const tileSize = 2.0;
const floorSize = 50 * tileSize;
const floorAnisotropy = renderer.capabilities.getMaxAnisotropy();
const floorTexture = new THREE.TextureLoader().load("./assets/floor_tile.png");
floorTexture.colorSpace = THREE.SRGBColorSpace;
floorTexture.repeat.set(floorSize / tileSize, floorSize / tileSize);
floorTexture.wrapS = floorTexture.wrapT = THREE.RepeatWrapping;
floorTexture.anisotropy = floorAnisotropy;
const floorMaterial = new THREE.MeshStandardMaterial({
    map: floorTexture,
    color: 0x404040,
    roughness: 0.9
});
const floorGeometry = new THREE.PlaneGeometry(floorSize, floorSize);
const floor = new THREE.Mesh(floorGeometry, floorMaterial);
floor.rotation.x = -Math.PI / 2;
floor.position.y = 0;
floor.receiveShadow = true;
scene.add(floor);

//add lighting
scene.add(new THREE.AmbientLight(0xf0f0f0, 7.0));
const light = new THREE.PointLight(0xffffff, 11.0);
light.position.set(0 + 0.4, 0 + 1.5, 0);
light.castShadow = true;
light.shadow.mapSize.width = 512;
light.shadow.mapSize.height = 512;
light.shadow.camera.near = 0.05;
light.shadow.camera.far = 50;
scene.add(light);

//load WASM functions
const simulate = Module.cwrap('simulate', 'void', ['number']);
const set_pilot_input = Module.cwrap('set_pilot_input', 'void', ['number', 'number', 'number', 'number']);
const retrieve_state_variable = Module.cwrap('retrieve_state_variable', 'number', ['int']);
const retrieve_simulation_time = Module.cwrap('retrieve_simulation_time', 'number', ['void']);

//allocate useful variables
const pilot_input = [0.5, 0.5, 0.3, 0.5];
let Cb = 5000.0;
let phi = 0.0;
let theta = 0.0;
let psi = 0.0;
let x = 0.0;
let y = 0.0;
let z = 0.0;
let Ux = 0.0;
let Uy = 0.0;
let Uz = 0.0;
let Uinf = 0.0;
let simulation_time = 0.0;

//load quadcopter
const loader = new STLLoader();
loader.load("./assets/f450_quadcopter_lowpoly.stl", (quadcopterGeometry) => {
    const quadcopterMaterial = new THREE.MeshPhongMaterial({ vertexColors: true, opacity: quadcopterGeometry.alpha });
    const quadcopter = new THREE.Mesh(quadcopterGeometry, quadcopterMaterial);
    quadcopter.scale.set(1e-3, 1e-3, 1e-3);
    quadcopter.rotation.x = -Math.PI / 2;
    quadcopter.position.y = 0.0;
    quadcopter.castShadow = true;
    quadcopter.receiveShadow = true;
    scene.add(quadcopter);
    controls.target = new THREE.Vector3(quadcopter.position.x, quadcopter.position.y, quadcopter.position.z);
    renderer.render(scene, camera);

    //animate scene
    let lastFrameTime = 0;
    renderer.setAnimationLoop((timestamp) => {
        if (paused) {
            lastFrameTime = timestamp;
            return;
        }
        if (timestamp - lastFrameTime < 1 / 30) {
            //limit to 30fps
            return;
        }
        const dt = (timestamp - lastFrameTime) / 1000;
        lastFrameTime = timestamp;

        //update quadcopter position
        set_pilot_input(pilot_input[0], pilot_input[1], pilot_input[2], pilot_input[3]);
        simulate(dt);
        Cb = retrieve_state_variable(0) / 3.6;
        phi = retrieve_state_variable(5);
        theta = retrieve_state_variable(6);
        psi = retrieve_state_variable(7);
        x = retrieve_state_variable(8);
        y = retrieve_state_variable(9);
        z = retrieve_state_variable(10);
        Ux = retrieve_state_variable(14);
        Uy = retrieve_state_variable(15);
        Uz = retrieve_state_variable(16);
        Uinf = Math.sqrt(Ux ** 2 + Uy ** 2 + Uz ** 2);
        simulation_time = retrieve_simulation_time();
        quadcopter.rotation.set(-theta - 3.14159 / 2, phi, psi, "XYZ");
        floor.position.x = y % (floorSize / 2);
        floor.position.y = -z;
        floor.position.z = x % (floorSize / 2);

        //periodic terrain boundaries
        if (floor.position.x < -floorSize / 4) {
            floor.position.x += floorSize / 2;
        }
        if (floor.position.x > floorSize / 4) {
            floor.position.x -= floorSize / 2;
        }
        if (floor.position.z < -floorSize / 4) {
            floor.position.z += floorSize / 2;
        }
        if (floor.position.z > floorSize / 4) {
            floor.position.z -= floorSize / 2;
        }

        //render scene
        camera.lookAt(quadcopter.position);
        controls.update();
        renderer.render(scene, camera);
        stats.update();

        //update Primary Flight Display // Changing this script does not seem to change anything. Depricated?
        const timeOverlayMinutes = (Math.floor(simulation_time / 60)).toString().padStart(2, '0');
        const timeOverlaySeconds = (Math.floor(simulation_time % 60)).toString().padStart(2, '0');
        document.getElementById("timeOverlay").innerText = timeOverlayMinutes + ":" + timeOverlaySeconds;
        document.getElementById("batteryOverlay").innerText = "🔋 " + Math.round(100 * Cb / 5000).toString() + "%";
        document.getElementById("altitudeIndicator").innerText = z.toFixed(1).toString().padStart(5, '0') + "m";
        document.getElementById("speedIndicator").innerText = Math.round(Uinf).toString().padStart(2, '0') + "km/h";
        // Ember's additions 

        /*
        (number)
        Converts radians to degrees. No Modulo operation is applied.
        */
        const radiansToDegrees = (radians) => (radians / (2 * Math.PI) * 360);

        document.getElementById("gyroIndicatorRoll").innerText = "_Roll: " + Math.round(radiansToDegrees(phi)).toString().padStart(3, '0') + "*";
        document.getElementById("gyroIndicatorPitch").innerText = "Pitch: " + Math.round(radiansToDegrees(theta)).toString().padStart(3, '0') + "*";
        gyroIndicatorYaw.innerText = "__Yaw: " + Math.round(radiansToDegrees(psi)).toString().padStart(3, '0') + "*";

        /*
        (number)
        */
        const kmHtoMS = (kmPerHour) => (kmPerHour * (5 / 18))

        document.getElementById("accelIndicatorX").innerText = "X: " + (kmHtoMS(Ux)).toPrecision(3).toString().padStart(3, '0') + "m/s";
        document.getElementById("accelIndicatorY").innerText = "Y: " + (kmHtoMS(Uy)).toPrecision(3).toString().padStart(3, '0') + "m/s";
        document.getElementById("accelIndicatorZ").innerText = "Z: " + (kmHtoMS(Uz)).toPrecision(3).toString().padStart(3, '0') + "m/s";
    });
});


//pause if running, or resume if paused
function pauseResume() {
    paused = !paused;
    if (paused) {
        renderer.render(scene, camera);
        document.getElementById("pauseOverlay").style.display = "block";
        document.getElementById("timeOverlay").classList.add("paused");
        document.getElementById("batteryOverlay").classList.add("paused");
        document.getElementById("flightModeOverlay").classList.add("paused");
        document.getElementById("altitudeOverlay").classList.add("paused");
        document.getElementById("speedOverlay").classList.add("paused");
    }
    else {
        document.getElementById("pauseOverlay").style.display = "none";
        document.getElementById("timeOverlay").classList.remove("paused");
        document.getElementById("batteryOverlay").classList.remove("paused");
        document.getElementById("flightModeOverlay").classList.remove("paused");
        document.getElementById("altitudeOverlay").classList.remove("paused");
        document.getElementById("speedOverlay").classList.remove("paused");
    }
}


//handle window resize event
window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
    if (paused) {
        renderer.render(scene, camera);
    }
});

//handle keyboard down event
document.addEventListener("keydown", (event) => {
    if (event.key === "Escape" || event.key === "p" || event.key === "P") {
        pauseResume();
    }

    if (event.key === "d" || event.key === "D") {
        pilot_input[0] = 0.6;
    }
    if (event.key === "a" || event.key === "A") {
        pilot_input[0] = 0.4;
    }
    if (event.key === "w" || event.key === "W") {
        pilot_input[1] = 0.6;
    }
    if (event.key === "s" || event.key === "S") {
        pilot_input[1] = 0.4;
    }
    if (event.key === "ArrowUp") {
        pilot_input[2] = 0.8;
    }
    if (event.key === "ArrowDown") {
        pilot_input[2] = 0.2;
    }
    if (event.key === "ArrowRight") {
        pilot_input[3] = 0.6;
    }
    if (event.key === "ArrowLeft") {
        pilot_input[3] = 0.4;
    }
});

//handle keyboard up event
document.addEventListener("keyup", (event) => {
    if (event.key === "d" || event.key === "D") {
        pilot_input[0] = 0.5;
    }
    if (event.key === "a" || event.key === "A") {
        pilot_input[0] = 0.5;
    }
    if (event.key === "w" || event.key === "W") {
        pilot_input[1] = 0.5;
    }
    if (event.key === "s" || event.key === "S") {
        pilot_input[1] = 0.5;
    }
    if (event.key === "ArrowUp" || event.key === "ArrowDown") {
        pilot_input[2] = 0.3;
    }
    if (event.key === "ArrowLeft" || event.key === "ArrowRight") {
        pilot_input[3] = 0.5;
    }
});

//handle "Resume Flight" menu button click event
document.getElementById("resumeFlight").addEventListener("click", () => {
    pauseResume();
});

//handle "Credits" menu button click event
document.getElementById("creditsButton").addEventListener("click", () => {
    document.getElementById("creditsWindow").style.display = "block";
});
document.getElementById("creditsWindowCloseButton").addEventListener('click', () => {
    document.getElementById("creditsWindow").style.display = "none";
});
