//------------------------------------------------------------------------------
//  A basic quadcopter simulator built using Three.js and C (WASM).
//  This module implements user input logic, rendering and communication with
//  the WASM module.
//
//  Author: Andrea Pavan
//  License: MIT
//------------------------------------------------------------------------------
import * as THREE from "./threejs/three.module.min.js";
import {OrbitControls} from "./threejs/addons/controls/OrbitControls.js";
import Stats from "./threejs/addons/libs/stats.module.js";
import {STLLoader} from "./threejs/addons/loaders/STLLoader.js";


//setup scene, camera and renderer
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf2f2f2);
scene.fog = new THREE.Fog(scene.background, 5, 50);
const camera = new THREE.PerspectiveCamera(70, window.innerWidth/window.innerHeight, 1, 10000);
camera.position.set(0, 6, 5);
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
controls.maxPolarAngle = Math.PI/2 - 0.1;
controls.listenToKeyEvents(window);

//draw ground
const floorGeometry = new THREE.PlaneGeometry(200, 200);
const floorMaterial = new THREE.ShadowMaterial({opacity: 0.2});
//const floorMaterial = new THREE.MeshPhongMaterial({color: 0x525252, flatShading: true});
const floor = new THREE.Mesh(floorGeometry, floorMaterial);
floor.rotation.x = -Math.PI/2;
floor.position.y = 0;
floor.receiveShadow = true;
scene.add(floor);
const floorGridHelper = new THREE.GridHelper(200, 200);
floorGridHelper.position.y = 0+1e-6;
floorGridHelper.material.opacity = 0.45;
floorGridHelper.material.transparent = true;
scene.add(floorGridHelper);

//add lighting
scene.add(new THREE.AmbientLight(0xf0f0f0, 3));
const light = new THREE.PointLight(0xffffff, 4.5);
light.position.set(0+4, 3+10, 0);
light.castShadow = true;
light.shadow.mapSize.width = 512;
light.shadow.mapSize.height = 512;
light.shadow.camera.near = 0.5;
light.shadow.camera.far = 500;
scene.add(light);

//load WASM functions
const simulate = Module.cwrap('simulate', 'void', []);
const set_pilot_input = Module.cwrap('set_pilot_input', 'void', ['number','number','number','number']);
const retrieve_state_variable = Module.cwrap('retrieve_state_variable', 'number', ['int']);

//allocate useful variables
const pilot_input = [0.5, 0.5, 0.4, 0.5];
let phi = 0;
let theta = 0;
let psi = 0;
let x = 0;
let y = 0;
let z = 0;

//load quadcopter
const loader = new STLLoader();
loader.load("./assets/f450_quadcopter_lowpoly.stl", (quadcopterGeometry) => {
    const quadcopterMaterial = new THREE.MeshPhongMaterial({vertexColors: true, opacity: quadcopterGeometry.alpha});
    const quadcopter = new THREE.Mesh(quadcopterGeometry, quadcopterMaterial);
    quadcopter.scale.set(1e-3, 1e-3, 1e-3);
    quadcopter.rotation.x = -Math.PI/2;
    quadcopter.position.y = 0.3;
    quadcopter.castShadow = true;
    quadcopter.receiveShadow = true;
    scene.add(quadcopter);
    controls.target = new THREE.Vector3(quadcopter.position.x, quadcopter.position.y, quadcopter.position.z);

    //animate the scene
    let lastFrameTime = 0;
    renderer.setAnimationLoop((timestamp) => {
        if (paused || timestamp - lastFrameTime < 25) {
            //limit to 40fps
            return;
        }
        lastFrameTime = timestamp;
        
        //update quadcopter position
        set_pilot_input(pilot_input[0], pilot_input[1], pilot_input[2], pilot_input[3]);
        simulate();
        phi = retrieve_state_variable(5);
        theta = retrieve_state_variable(6);
        psi = retrieve_state_variable(7);
        x = retrieve_state_variable(8);
        y = retrieve_state_variable(9);
        z = retrieve_state_variable(10);
        quadcopter.rotation.set(theta-3.14159/2, phi, psi, "XYZ");
        floor.position.x = y;
        floor.position.y = -z;
        floor.position.z = -x;
        floorGridHelper.position.x = floor.position.x;
        floorGridHelper.position.y = floor.position.y+1e-6;
        floorGridHelper.position.z = floor.position.z;

        //periodic terrain boundaries
        if (floor.position.x < -50) {
            floor.position.x += 100;
        }
        if (floor.position.x > 50) {
            floor.position.x -= 100;
        }
        if (floor.position.z < -50) {
            floor.position.z += 100;
        }
        if (floor.position.z > 50) {
            floor.position.z -= 100;
        }

        //rendering code
        camera.lookAt(quadcopter.position);
        controls.update();
        renderer.render(scene, camera);
        stats.update();
    });
});


//handle window resize event
window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

//handle escape button event
document.addEventListener("keydown", (event) => {
    if (event.key === "Escape" || event.key === "p" || event.key === "P") {
        paused = !paused;
        if (paused) {
            document.getElementById("pauseOverlay").style.display = "block";
        }
        else {
            document.getElementById("pauseOverlay").style.display = "none";
        }
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
        pilot_input[2] = 0.4;
    }
    if (event.key === "ArrowLeft" || event.key === "ArrowRight") {
        pilot_input[3] = 0.5;
    }
});

