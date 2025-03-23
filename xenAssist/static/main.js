import { updateSceneFromData, highlightHex } from './scene.js'; 

// Grab references to DOM elements
const menuOverlay = document.getElementById('menu-overlay');
const toggleMenuBtn = document.getElementById('toggle-menu');
const menuItems = document.querySelectorAll('#menu-overlay li');
const pitchClassSwitch = document.getElementById('pitchClassSwitch');
const intervalSwitch = document.getElementById('intervalSwitch');
const statusBar = document.getElementById('status-bar');

const activeNotes = new Map();
const savedNotes = new Map();

let selectedMenuItem = null;
let lastLayout = null;

// 1) Toggle the menu on button click
toggleMenuBtn.addEventListener('click', () => {
	if (event.type.startsWith('mouse') && event.button !== 0) return;
  menuOverlay.classList.toggle('show');
});

// A function to fetch layout from server + update the scene
function fetchAndApplyLayout(index) {	
  // e.g. GET /getLayout?index=...
  fetch('/getLayout?index=' + index)
    .then(r => r.json())
    .then(data => {
      console.log("Fetched layout for index", index, ":", data);	  
	  // reset active notes view
	  activeNotes.clear();
	  savedNotes.clear();
	  showNotes();
      // Let the scene code do the actual update
      updateSceneFromData(data, pitchClassSwitch.checked, intervalSwitch.checked, true);
	  lastLayout = data;
	  
      fetch('/sendSavedNotes', {
        method: 'GET',
        cache: 'no-cache'
      }).catch(err => console.error('Error in sendSavedNotes:', err));
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


function setStatusText(text, textColor) {
  const bar = document.getElementById('status-bar');
  const textEl = document.getElementById('status-bar-text');

  if (!text || text.trim() === '') {
    // Hide if empty
    bar.style.display = 'none';
    return;
  }

  // Show the bar
  bar.style.display = 'block';
  
  // Ensure textColor is in valid CSS hex format
  if (textColor && textColor[0] !== '#') {
    textColor = '#' + textColor;
  }

  // Set the foreground color of the text
  textEl.style.color = textColor;
  // Set the text content
  textEl.textContent = text;

  // Reset to a big font size to start (we'll shrink if needed)
  let fontSize = 120; 
  textEl.style.fontSize = fontSize + 'px';
  textEl.style.marginTop = (fontSize / 3.8) + 'px';
	  
  console.log("scrollWidth: ", textEl.scrollWidth);
  console.log("clientWidth: ",  bar.clientWidth);

  // Auto-scale down if the text is too wide
  while (textEl.scrollWidth > bar.clientWidth * 0.95 && fontSize > 10) {
    fontSize--;
    textEl.style.fontSize = fontSize + 'px';
  }
}

function getLabels(notes) {
	const notesToShow = [...notes]
	const lowestPitch = Math.min(...notesToShow.map(n => n["Pitch"] ));
	
	const labelsToShow = notesToShow.map(n => {	
		if (pitchClassSwitch.checked) {
			if (intervalSwitch.checked) {
				return (n["Pitch"] - lowestPitch).toString();
			} else {
				return n["Class"] + "(" + n["Base"] + ")";
			}
		} else { 
			if (intervalSwitch.checked) {
				return (n["Pitch"] - lowestPitch).toString();
			} else {
				return n["NoteUnicode"];
			}
		}
	});
	
	
    if (intervalSwitch.checked)
		return labelsToShow.join('-');
    else		
		return labelsToShow.join(' ');
}

function showNotes() {
	const activeLabels = getLabels(activeNotes.values());
	const savedLabels = getLabels(savedNotes.values());	
	
	if (activeLabels) setStatusText(activeLabels, "ffffff");
	else setStatusText(savedLabels, "ffcc00");
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
  let [keys, color, isActive] = args;
  console.log('Server sent noteOn:', keys, color);
  const notes = highlightHex(keys, color, pitchClassSwitch.checked, intervalSwitch.checked);
  
  notes.forEach(note => {
	  if (isActive && !activeNotes.has(note["Note"])) {
		activeNotes.set(note["Note"], note);
	  }
	  else if (!isActive && !savedNotes.has(note["Note"])) {
		savedNotes.set(note["Note"], note);
	  }
  });
  
  showNotes();
});

socket.on('noteOff', (args) => { 
  let [keys, isActive] = args;
  console.log('Server sent noteOff:', keys);
  const notes = highlightHex(keys, null, pitchClassSwitch.checked, intervalSwitch.checked);
  
  if (isActive) 
	  notes.forEach(n => activeNotes.delete(n["Note"])); 
  else 
	  notes.forEach(n => savedNotes.delete(n["Note"])); 
  showNotes();
});

selectFirstItemInitially();

function handleSwitchChange(event) {
  if (!lastLayout) return;
  if (event.type.startsWith('mouse') && event.button !== 0) return;
  updateSceneFromData(lastLayout, pitchClassSwitch.checked, intervalSwitch.checked, false);
  showNotes();
}

pitchClassSwitch.addEventListener('change', handleSwitchChange);
intervalSwitch.addEventListener('change', handleSwitchChange);


function flashRegion(event) {
  // Create the flash element
  const flash = document.createElement("div");
  flash.classList.add("flash-effect");

  // Get the bounding rect of the status bar so we can position the flash correctly
  const rect = statusBar.getBoundingClientRect();
  flash.style.left = (event.clientX - rect.left) + "px";
  flash.style.top = (event.clientY - rect.top) + "px";

  // Append to the status bar container
  statusBar.appendChild(flash);

  // Remove the flash element once the animation is complete
  setTimeout(() => {
    flash.remove();
  }, 500); // matches the animation duration
}

	
function handleStatusBarEvent(event) {
  // Only act if the interval switch is checked and status bar is visible.
  //if (!intervalSwitch.checked) return;
  if (window.getComputedStyle(statusBar).display === 'none') return;
  event.preventDefault();
	
  // Show visual feedback for the click:
  flashRegion(event);

  // Get the bounding rectangle of the status bar.
  const rect = statusBar.getBoundingClientRect();
  const clickX = event.clientX - rect.left;

  // Define thresholds to split the bar into thirds.
  const leftThreshold = rect.width / 3;
  const rightThreshold = 2 * rect.width / 3;

  const pitchArray = Array.from(activeNotes.values()).map(note => note["Pitch"]);
  const pitchesStr = pitchArray.join(',');
  
  if (clickX < leftThreshold) {
    // Left third: call rotateInterval with direction -1.
    const url = `/rotateInterval?pitches=` + encodeURIComponent(JSON.stringify(pitchArray)) + `&direction=-1`;
    fetch(url, { method: 'GET', cache: 'no-cache' })
      .catch(err => console.error("rotateInterval down error:", err));
  } else if (clickX > rightThreshold) {
    // Right third: call rotateInterval with direction 1.
    const url = `/rotateInterval?pitches=` + encodeURIComponent(JSON.stringify(pitchArray)) + `&direction=1`;
    fetch(url, { method: 'GET', cache: 'no-cache' })
      .catch(err => console.error("rotateInterval up error:", err));
  } else {
    // Center third: build pitchArray from activeNotes and call saveInterval.
    const pitchArray = Array.from(activeNotes.values()).map(note => note["Pitch"]);
	console.log(activeNotes.values());
    const url = `/saveInterval?pitches=` + encodeURIComponent(JSON.stringify(pitchArray));
    fetch(url, { method: 'GET', cache: 'no-cache' })
      .catch(err => console.error("saveInterval error:", err));
  }
  activeNotes.clear();
  showNotes();
}

if (window.PointerEvent) {
  statusBar.addEventListener('pointerup', handleStatusBarEvent);
} else {
  statusBar.addEventListener('click', handleStatusBarEvent);
  statusBar.addEventListener('touchend', handleStatusBarEvent);
}
