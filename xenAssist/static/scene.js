import * as THREE from 'https://unpkg.com/three@latest/build/three.module.js';
import { OrbitControls } from 'https://unpkg.com/three@latest/examples/jsm/controls/OrbitControls.js';
import { FontLoader } from 'https://unpkg.com/three@latest/examples/jsm/loaders/FontLoader.js';
import { TextGeometry } from 'https://unpkg.com/three@latest/examples/jsm/geometries/TextGeometry.js';

const hexagons = {};
const layout = {};
let appFont = null;   // global in this file
	
const pattern = [
	[ 1, 2, 0, 0, 0, 0],  // First row: two hexagons
	[ 3, 4, 5, 6, 7, 0],  // Second row: five hexagons
	[ 8, 9,10,11,12,13],  // Repeat first row
	[14,15,16,17,18,19],  // Repeat second row
	[20,21,22,23,24,25],
	[26,27,28,29,30,31],
	[32,33,34,35,36,37],
	[38,39,40,41,42,43],
	[44,45,46,47,48,49],
	[ 0,50,51,52,53,54],
	[ 0, 0, 0, 0,55,56]
];
const patternKeys = 56; // Kezs per pattern  
const repeatCount = 5; // Number of times to repeat the pattern to the right

// Automatically determine pattern size
const patternRows = pattern.length;  // Number of rows in the base pattern
const patternCols = pattern[0].length; // Number of cols in the base pattern

// Calculate final grid size

const rows = patternRows + (repeatCount * 2); // Extra rows due to downward shift
const cols = patternCols * repeatCount + 1; // Total columns after horizontal repetition

// Create the full pattern
const fullPattern = Array.from({ length: rows }, () => Array(cols).fill(0)); // Initialize empty grid

// Populate the full pattern
for (let rep = 0; rep < repeatCount; rep++) { // Repeat to the right
	for (let r = 0; r < patternRows; r++) {   // Iterate over pattern rows
		for (let c = 0; c < patternCols; c++) { // Iterate over pattern cols
			if (pattern[r][c] > 0) { // Only copy hexagons
				const newRow = r + (rep * 2); // Shift one row down per repetition
				let newCol = c + (rep * patternCols + 1); // Offset horizontally
				
				if (newRow < rows) { // Prevent overflow
					fullPattern[newRow][newCol] = pattern[r][c] + rep * patternKeys;
				}
			}
		}
	}
}


// Scene Setup
const scene = new THREE.Scene();
//const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 1000);
const aspect = window.innerWidth / window.innerHeight;
const cameraSize = Math.tan((50 * Math.PI) / 360) * 15; // Approximate conversion

const zoomLevel = 1;
const camera = new THREE.OrthographicCamera(
	(-cameraSize * aspect / zoomLevel),  // left
	 (cameraSize * aspect) / zoomLevel,  // right
	 cameraSize / zoomLevel,           // top
	-cameraSize / zoomLevel,           // bottom
	0.1,                   // near
	1000                   // far
);

// Position the camera
camera.lookAt(0, 0, 0);
camera.position.set(0, 0, 15); // View from above

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
document.body.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;
controls.minDistance = 5;
controls.maxDistance = 30;
controls.panSpeed = 1.0; 
controls.enablePan = true;
controls.enableRotate = false;
controls.screenSpacePanning = true;
controls.mouseButtons = {
  LEFT: undefined,    // Left mouse button pans
  MIDDLE: THREE.MOUSE.DOLLY, // Middle mouse button zooms/dolies
  RIGHT: THREE.MOUSE.PAN     // Right mouse button also pans
};

renderer.domElement.addEventListener('contextmenu', event => event.preventDefault());


// Lighting
const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 1.5);
directionalLight.position.set(5, 10, 5);
directionalLight.castShadow = true;
scene.add(directionalLight);

// Font Loader for Letters
const fontLoader = new FontLoader();
fontLoader.load('https://threejs.org/examples/fonts/helvetiker_regular.typeface.json', (font) => {
  appFont = font;
  console.log("Font loaded!");
  // Now you can create your grid of hexagons, etc.
});
	
/**
 * Creates a text mesh for a given letter/string, using a given font and color.
 * Returns the Mesh so you can add() it to a hex.
 */
function createTextMesh(letter, color = 0x000000) {
  if (!appFont) {
	//console.warn("Font not loaded yet. Can't create text.");
	return null;
  }

  const textGeometry = new TextGeometry(letter, {
	font: appFont,
	size: 0.3,
	depth: 0.1,
	curveSegments: 6,
	bevelEnabled: false
  });
  textGeometry.center();

  const textMaterial = new THREE.MeshStandardMaterial({ color });
  const textMesh = new THREE.Mesh(textGeometry, textMaterial);

  // Adjust rotations/position as you had before
  textMesh.rotation.y = 2 * Math.PI;
  textMesh.rotation.z = 3 * Math.PI / 2;
  textMesh.position.z = 0.56;

  return textMesh;
}

/**
 * Removes the old text mesh (if any) and attaches a new one
 */
function updateHexText(hex, newLetter, color = 0x000000) {
  // 1) Find the old text child if it exists
  let oldTextMesh = null;
  for (let i = 0; i < hex.children.length; i++) {
	if (hex.children[i].isMesh && hex.children[i].geometry instanceof TextGeometry) {
	  oldTextMesh = hex.children[i];
	  break;
	}
  }

  // 2) Remove the old text
  if (oldTextMesh) {
	hex.remove(oldTextMesh);
	oldTextMesh.geometry.dispose();  // free GPU memory
	oldTextMesh.material.dispose();
  }

  // 3) Create a new text mesh
  const newTextMesh = createTextMesh(newLetter, color);
  
  // 4) Attach it to the hex
  if (newTextMesh)
	hex.add(newTextMesh);
}


// Function to create a hexagon with a letter
function createHexagon(letter) {
	const shape = new THREE.Shape();
	const radius = 1;
	for (let i = 0; i < 6; i++) {
		const angle = (i / 6) * Math.PI * 2;
		const x = Math.cos(angle) * radius;
		const y = Math.sin(angle) * radius;
		if (i === 0) {
			shape.moveTo(x, y);
		} else {
			shape.lineTo(x, y);
		}
	}
	shape.closePath();

	const extrudeSettings = { depth: 0.5, bevelEnabled: true, bevelSize: 0.05, bevelThickness: 0.1 };
	const geometry = new THREE.ExtrudeGeometry(shape, extrudeSettings);
	const material = new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.1, roughness: 0.5 });

	const hexagon = new THREE.Mesh(geometry, material);
	hexagon.castShadow = true;
	hexagon.receiveShadow = true;
	hexagon.rotation.x = Math.PI / 2;
	hexagon.rotation.z = Math.PI / 2;

	// Create and attach the text
	const textMesh = createTextMesh(letter, 0xff0000);
	if (textMesh)
		hexagon.add(textMesh);

	return hexagon;
}

// Create a grid of hexagons with letters
const hexWidth = 2 * Math.cos(Math.PI / 6) + 0.05; // Hexagon width
const hexHeight = 1.5 + 0.1; // Vertical distance between rows

const hexGrid = new THREE.Group(); // Create a group for the whole grid
scene.add(hexGrid); // Add it to the scene

for (let row = 0; row < rows; row++) {
	for (let col = 0; col < cols; col++) {
		if (fullPattern[row][col] === 0) continue;
		const randomLetter = String(fullPattern[row][col])
		const hex = createHexagon(randomLetter);
		hexGrid.add(hex);
		hexagons[fullPattern[row][col]-1] = hex;

		// Offset calculation for honeycomb structure
		const xOffset = col * hexWidth * 1.1; // Horizontal spacing
		const yOffset = row * hexHeight; // Vertical spacing

		if (row % 2 !== 0) {
			hex.position.x = xOffset + hexWidth * 0.55; // Shift odd rows
		} else {
			hex.position.x = xOffset;
		}

		hex.position.y = 0;
		hex.position.z = -yOffset; // Move along z-axis to form the grid
	}
}

hexGrid.rotation.x = 3 * Math.PI / 2; // Rotate the grid to be face-on
hexGrid.rotation.y = -Math.PI / 12.5;
			hexGrid.position.x = -cols * hexWidth / 1.9;
hexGrid.position.y = (rows * hexHeight / 2) * 0.5;

// Raycaster for interaction
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();

// Helper to return an array of points from the event
function getEventPoints(event) {
  let points = [];
  if (event.type.startsWith('touch')) {
    // For touch events, iterate over changedTouches
    for (let touch of event.changedTouches) {
      points.push({
        clientX: touch.clientX,
        clientY: touch.clientY,
        id: touch.identifier
      });
    }
  } else {
    // For mouse events, use one point with a fixed id ("mouse")
    points.push({
      clientX: event.clientX,
      clientY: event.clientY,
      id: 'mouse'
    });
  }
  return points;
}

function pickHex(event) {
  if (event.type.startsWith('mouse') && event.button !== 0) return;
  if (event.type.startsWith('touch')) {
    event.preventDefault();
  }
  let points = getEventPoints(event);
  points.forEach(point => {
    mouse.x = (point.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(point.clientY / window.innerHeight) * 2 + 1;
    raycaster.setFromCamera(mouse, camera);

    const intersects = raycaster.intersectObjects(Object.values(hexagons), false);
    if (intersects.length > 0) {
      let pressedHex = intersects[0].object;
      let hexKey = Object.entries(hexagons).find(([key, val]) => val === pressedHex)?.[0];
      const keyString = layout[hexKey]["Key"] + '@' + (layout[hexKey]["Chan"] - 1);
	  
      console.log("Sending noteOn for " + keyString);
      fetch('/noteOn?keys=' + encodeURIComponent(keyString) + '&color=' + encodeURIComponent("ffffff"), {
        method: 'GET',
        cache: 'no-cache'
      }).catch(err => console.error('Error in noteOn:', err));
      
      // Store the pressed hex using the point's unique id
      if (point.id === 'mouse') {
        activeMouseHex = { hex: pressedHex, key: hexKey };
      } else {
        activeHexes.set(point.id, { hex: pressedHex, key: hexKey });
      }
    }
  });

  // Hide menu if open
  if (menuOverlay.classList.contains('show')) {
    menuOverlay.classList.remove('show');
  }
}

function sendNoteOff(hexKey) {
  const keyString = layout[hexKey]["Key"] + '@' + (layout[hexKey]["Chan"] - 1);
  console.log("Sending noteOff for " + keyString);
  fetch('/noteOff?keys=' + encodeURIComponent(keyString), {
    method: 'GET',
    cache: 'no-cache'
  }).catch(err => console.error('Error in noteOff:', err));
}


function letgoHex(event) {
  if (event.type.startsWith('mouse') && event.button !== 0) return;
  let points = getEventPoints(event);
  points.forEach(point => {
    let hexKey;
    if (point.id === 'mouse') {
      if (activeMouseHex) {
        hexKey = activeMouseHex.key;
        activeMouseHex = null;
      }
    } else {
      if (activeHexes.has(point.id)) {
        hexKey = activeHexes.get(point.id).key;
        activeHexes.delete(point.id);
      }
    }
    if (hexKey !== undefined) {
      sendNoteOff(hexKey);
    }
  });
}


// Global variables for tracking active hex presses:
let activeHexes = new Map(); // For touch events: key = touch.identifier, value = { hex, key }
let activeMouseHex = null;   // For mouse events

// Attach event listeners
renderer.domElement.addEventListener('mousedown', pickHex);
renderer.domElement.addEventListener('touchstart', pickHex);
renderer.domElement.addEventListener('mouseup', letgoHex);
renderer.domElement.addEventListener('touchend', letgoHex);


// Animation Loop
function animate() {
	requestAnimationFrame(animate);
	controls.update();
	renderer.render(scene, camera);
}
animate();

// Resize Handling
window.addEventListener('resize', () => {
	camera.aspect = window.innerWidth / window.innerHeight;
	camera.updateProjectionMatrix();
	renderer.setSize(window.innerWidth, window.innerHeight);
});


// ===== Toggle Menu Logic =====
const toggleMenuBtn = document.getElementById('toggle-menu');
const menuOverlay = document.getElementById('menu-overlay');

// Toggle the menu on button click
toggleMenuBtn.addEventListener('click', () => {
  menuOverlay.classList.toggle('show');
});

window.addEventListener('resize', onWindowResize, false);

function onWindowResize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  const aspect = width / height;
  
  // Update orthographic camera boundaries using your original values
  camera.left   = -cameraSize * aspect / zoomLevel;
  camera.right  = cameraSize * aspect / zoomLevel;
  camera.top    = cameraSize / zoomLevel;
  camera.bottom = -cameraSize / zoomLevel;
  camera.updateProjectionMatrix();
  
  // Update the renderer
  renderer.setSize(width, height);
  renderer.setPixelRatio(window.devicePixelRatio);
  
  // Render the scene (if not using a continuous loop)
  renderer.render(scene, camera);
}

export function updateSceneFromData(layoutData, pitchClassView, intervalView, updateBackground) {
  // Example usage of layoutData
  //const color = layoutData.hexData.color || "#ffffff";
  //const textOverride = layoutData.hexData.textOverride || "DEF";

  // For demonstration, let's recolor *all* hexagons to the same color:
  Object.keys(hexagons).forEach(key => {
	  const hex = hexagons[key];
	  const board = Math.floor(key / patternKeys); // integer division
	  const controlNumber = key % patternKeys;
	  //console.log("Key " + key + " Board "+board+ " Control "+controlNumber);
	  layout[key] = layoutData["Board"+board][controlNumber]; 
	  if (updateBackground)
		hex.material.color.set("#"+layout[key]["Col"].slice(-6));
	  if (pitchClassView)
		updateHexText(hex, layout[key]["Class"].toString(), appFont, 0x000000);
	  else
		updateHexText(hex, layout[key]["Note"], appFont, 0x000000);
  });
};

export function highlightHex(keys, color) {  
  const notes = [];
  Object.keys(hexagons).forEach(key => {
	  const hex = hexagons[key];
	  keys.forEach(([keyToHighlight, channelToHighlight]) => {
		//console.log(`Key: ${layout[key]["Key"]}, Channel: ${layout[key]["Chan"]}`);
		//console.log(`Key: ${String(keyToHighlight)}, Channel: ${String(channelToHighlight)}`);
		if (layout[key]["Chan"] === (channelToHighlight+1) && layout[key]["Key"] === keyToHighlight) {
			console.log(`Key: ${keyToHighlight}, Channel: ${channelToHighlight} set to ${color}`);
			if (!color)
				hex.material.color.set("#"+layout[key]["Col"].slice(-6));
			else 
				hex.material.color.set("#"+color.slice(-6));
			notes.push(layout[key]);
		}
	  });
  })
  return notes;
}