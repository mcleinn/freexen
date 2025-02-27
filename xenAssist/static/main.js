import { updateSceneFromData, highlightHex } from './scene.js'; 

// Grab references to DOM elements
const menuOverlay = document.getElementById('menu-overlay');
const toggleMenuBtn = document.getElementById('toggle-menu');
const menuItems = document.querySelectorAll('#menu-overlay li');

const activeNotes = new Set();

let selectedMenuItem = null;

// 1) Toggle the menu on button click
toggleMenuBtn.addEventListener('click', () => {
  menuOverlay.classList.toggle('show');
});

// A function to fetch layout from server + update the scene
function fetchAndApplyLayout(index) {
  // e.g. GET /getLayout?index=...
  fetch('/getLayout?index=' + index)
    .then(r => r.json())
    .then(data => {
      console.log("Fetched layout for index", index, ":", data);
      // Let the scene code do the actual update
      updateSceneFromData(data);
    })
    .catch(err => console.error(err));
}

// === 1) At initial load, select first item + fetch layout ===
function selectFirstItemInitially() {
  if (menuItems.length > 0) {
    // Mark it as selected (UI highlight)
    menuItems[7].classList.add('selected');
    selectedMenuItem = menuItems[7];

    // Also fetch layout for index=0
    fetchAndApplyLayout(7);
  }
}


function setStatusText(text) {
  const bar = document.getElementById('status-bar');
  const textEl = document.getElementById('status-bar-text');

  if (!text || text.trim() === '') {
    // Hide if empty
    bar.style.display = 'none';
    return;
  }

  // Show the bar
  bar.style.display = 'block';

  // Set the text content
  textEl.textContent = text;

  // Reset to a big font size to start (we'll shrink if needed)
  let fontSize = 160; 
  textEl.style.fontSize = fontSize + 'px';
	  
  console.log("scrollWidth: ", textEl.scrollWidth);
  console.log("clientWidth: ",  bar.clientWidth);

  // Auto-scale down if the text is too wide
  while (textEl.scrollWidth > bar.clientWidth * 0.95 && fontSize > 10) {
    fontSize--;
    textEl.style.fontSize = fontSize + 'px';
  }
}

function showActiveNotes() {
	setStatusText([...activeNotes].join(' '));
}


// 2) When a menu item is clicked, highlight it, close the menu,
// and send an AJAX call to /menuSelection?index=...
menuItems.forEach((li, index) => {
  li.addEventListener('click', () => {
    // De-select the old item, if any
    if (selectedMenuItem) {
      selectedMenuItem.classList.remove('selected');
    }
    // Select new item
    li.classList.add('selected');
    selectedMenuItem = li;

    // Close the menu
    menuOverlay.classList.remove('show');

    // Send Ajax (GET)
    fetch('/menuSelection?index=' + index)
      .then(res => res.text())
      .then(txt => console.log('Server response:', txt))
      .catch(err => console.error('Error:', err));
	  
	// Now fetch layout for the chosen index
    fetchAndApplyLayout(index);
  });
});

console.log("Attaching menu click listener...");
toggleMenuBtn.addEventListener('click', () => {
  console.log("Menu button clicked!");
  menuOverlay.classList.toggle('show');
});

// ============ SOCKET.IO (WebSocket) ============ 
const socket = io(); // connects to the same origin:port by default

socket.on('connect', () => {
  console.log('Socket connected!');
});

// If the server forces a menu item, highlight it
socket.on('changeScale', (scaleIndex) => {
  console.log('Server forcing scale change:', scaleIndex);
  if (scaleIndex >= 0 && scaleIndex < menuItems.length) {
    if (selectedMenuItem) {
      selectedMenuItem.classList.remove('selected');
    }
    menuItems[scaleIndex].classList.add('selected');
    selectedMenuItem = menuItems[scaleIndex];
  }

	// Now fetch layout for the chosen index
	fetchAndApplyLayout(scaleIndex);
});

socket.on('noteOn', (args) => {
  let [keys, color] = args;
  console.log('Server sent noteOn:', keys, color);
  const notes = highlightHex(keys, color);
  
  notes.forEach(n => activeNotes.add(n)); 
  showActiveNotes();
});

socket.on('noteOff', (keys) => {
  console.log('Server sent noteOff:', keys);
  const notes = highlightHex(keys, null);
  
  notes.forEach(n => activeNotes.delete(n)); 
  showActiveNotes();
});

selectFirstItemInitially();
