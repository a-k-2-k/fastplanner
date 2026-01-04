# 🎵 VoxelFlow

<div align="center">

**Transform your music into mesmerizing 3D visualizations**

A real-time 3D audio visualizer that brings your music to life through a dynamic grid of reactive voxels.

[![Three.js](https://img.shields.io/badge/Three.js-r128-black?style=for-the-badge&logo=three.js)](https://threejs.org/)
[![Web Audio API](https://img.shields.io/badge/Web_Audio_API-Native-purple?style=for-the-badge)](https://developer.mozilla.org/en-US/docs/Web/API/Web_Audio_API)
[![No Dependencies](https://img.shields.io/badge/Dependencies-Zero-green?style=for-the-badge)](https://github.com/a-k-2-k/voxelflow)

[Demo](#) · [Features](#features) · [Technical Details](#technical-details)

</div>

---

## 🌟 Overview

VoxelFlow is an immersive 3D audio visualizer that transforms MP3 audio files into real-time visual experiences. Each of the 1,024 voxels (3D cubes) in a 32×32 grid represents a specific frequency band, pulsing and changing colors based on the music's amplitude and frequency distribution.

### ✨ What Makes VoxelFlow Special

- **Real-time Frequency Analysis** - Each voxel responds to a specific frequency band
- **Zero External Dependencies** - Pure vanilla JavaScript with only Three.js CDN
- **Single File Architecture** - Everything in one HTML file for easy deployment
- **Retro-Futuristic Aesthetic** - Neon colors and 80s synthwave vibes
- **Smooth 60fps Performance** - Optimized for fluid animations

---

## 🎮 Features

### Core Functionality

- 🎵 **MP3 Upload** - Drag-and-drop or click to upload
- 🎨 **1024 Reactive Voxels** - 32×32 grid responding to audio frequencies
- 🎭 **Dynamic Colors** - Hue spectrum from cyan to pink based on frequency
- 📹 **Auto-Orbiting Camera** - Smooth circular motion around the visualization
- ⏯️ **Playback Controls** - Play, pause, stop, and seek functionality
- ⏱️ **Time Display** - Current time and total duration
- 📊 **Progress Bar** - Visual progress with clickable seeking

### Visual Effects

- Voxel heights scale with frequency amplitude (0.5 to 15.5 units)
- Color mapping across the visible spectrum
- Emissive materials for glowing effects
- Smooth interpolation for fluid animations
- Fog effect for atmospheric depth
- Grid helper for retro-futuristic feel

---

## 🚀 Getting Started

### Quick Start

1. **Clone the repository:**
   ```bash
   git clone https://github.com/a-k-2-k/voxelflow.git
   cd voxelflow
   ```

2. **Open in browser:**
   ```bash
   open voxelflow.html
   # or simply double-click voxelflow.html
   ```

3. **Upload an MP3 file and enjoy!**

### Requirements

- **Modern web browser** with WebGL support (Chrome, Firefox, Safari, Edge)
- **Local MP3 files** to visualize
- **Mid-range hardware** recommended for smooth 60fps performance

---

## 🛠️ Technical Details

### Technology Stack

| Technology | Purpose | Version |
|------------|---------|---------|
| **Three.js** | 3D rendering engine | r128 |
| **Web Audio API** | Audio processing and frequency analysis | Native |
| **Vanilla JavaScript** | Application logic | ES6+ |
| **HTML5 & CSS3** | UI and styling | - |

### Architecture

```
┌─────────────────────────────────────────────────┐
│              MP3 Audio File                     │
└───────────────────┬─────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│         HTML5 Audio Element                     │
└───────────────────┬─────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│    Web Audio API (AudioContext)                 │
│    ├─ MediaElementSource                        │
│    └─ AnalyserNode (FFT: 2048)                  │
└───────────────────┬─────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│    Uint8Array[1024] Frequency Data              │
│    (0-255 values per frequency bin)             │
└───────────────────┬─────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│    1024 Three.js Voxel Meshes                   │
│    ├─ Scale.y (height) = f(amplitude)           │
│    └─ Color = f(frequency, amplitude)           │
└───────────────────┬─────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│    Three.js WebGL Renderer                      │
│    60fps via requestAnimationFrame              │
└─────────────────────────────────────────────────┘
```

### Key Implementation Details

#### Voxel Grid Configuration
- **Grid Size:** 32×32 = 1,024 voxels
- **Spacing:** 1.5 units between voxels
- **Geometry:** Shared `BoxGeometry(1, 1, 1)` for memory efficiency
- **Material:** Individual `MeshPhongMaterial` per voxel

#### Audio-to-Visual Mapping
```javascript
// Frequency → Height
targetHeight = 0.5 + (normalizedValue * 15)

// Frequency Index → Color Hue
hue = (voxelIndex / 1024) * 0.6  // Cyan to Pink spectrum

// Amplitude → Lightness
lightness = 0.5 + (normalizedValue * 0.3)
```

#### Performance Optimizations
- Shared geometry instance (saves ~1MB memory)
- Lerp smoothing (factor: 0.3) for fluid animations
- RequestAnimationFrame sync for optimal frame timing
- Scene fog for rendering efficiency
- No physics calculations required

---

## 🎨 Design Philosophy

### Retro-Futuristic Aesthetic

Inspired by 1980s sci-fi, Tron, and synthwave culture:

- **Color Palette:**
  - Primary: Neon Cyan (`#00fff2`)
  - Accent: Neon Pink (`#ff006e`)
  - Accent: Neon Purple (`#8b5cf6`)
  - Background: Dark Space (`#0a0e27`, `#050714`)

- **Typography:**
  - Headers: Orbitron (900 weight)
  - Mono: Share Tech Mono

- **Effects:**
  - Glowing text and UI elements
  - Backdrop blur on panels
  - Emissive 3D materials
  - Grid helper in 3D space

---

## 📁 Project Structure

```
voxelflow/
├── voxelflow.html      # Single-file application
│   ├── HTML structure
│   ├── CSS (inline)
│   └── JavaScript (inline)
└── README.md           # This file
```

### File Organization (within voxelflow.html)

```
HTML Structure:
├── #container
│   ├── #canvas-container (Three.js render target)
│   └── #ui-overlay
│       ├── header (VoxelFlow title)
│       ├── #instructions
│       └── #controls
│           ├── #drop-zone (file upload)
│           └── #playback (controls panel)

JavaScript Modules (conceptual):
├── Audio System (Web Audio API)
├── Three.js System (3D rendering)
└── UI System (event handlers)
```

---

## 🔧 Development

### Modifying the Codebase

#### Adding New Visualization Modes
```javascript
function updateVoxelsWaveMode() {
  analyser.getByteFrequencyData(dataArray);
  for (let i = 0; i < voxels.length; i++) {
    // Apply different transformations
    // e.g., wave patterns, circular arrangements
  }
}
```

#### Changing Grid Size
```javascript
const gridSize = 40; // Update from 32 (must be perfect square)
// Adjust camera distance proportionally
camera.position.set(0, 30, 50); // Increase for larger grids
```

#### Customizing Colors
```javascript
// Modify HSL hue range in updateVoxels()
const hue = (i / voxels.length) * 0.8; // Expand spectrum
const color = new THREE.Color().setHSL(hue, 1.0, lightness);
```

### Code Style

- **Indentation:** 4 spaces
- **Naming:** camelCase for variables and functions
- **Constants:** Use `const` for configuration values
- **Comments:** Explain "why" not "what"
- **Event Listeners:** Arrow functions preferred

### Testing Checklist

- [ ] File upload (drag-and-drop and click)
- [ ] Audio playback without errors
- [ ] Real-time voxel responsiveness
- [ ] Smooth camera rotation
- [ ] All playback controls functional
- [ ] Progress bar updates and seeking works
- [ ] Time display formatting
- [ ] Window resize handling
- [ ] No console errors
- [ ] 60fps performance maintained

---

## 🌈 Browser Compatibility

| Feature | Chrome | Firefox | Safari | Edge |
|---------|--------|---------|--------|------|
| Web Audio API | ✅ | ✅ | ✅ | ✅ |
| WebGL | ✅ | ✅ | ✅ | ✅ |
| File API | ✅ | ✅ | ✅ | ✅ |
| Drag & Drop | ✅ | ✅ | ✅ | ✅ |

**Minimum Versions:**
- Chrome 34+
- Firefox 25+
- Safari 9+
- Edge 12+

---

## 🚧 Known Limitations

- Only supports MP3 files (no WAV, OGG, FLAC)
- No mobile touch controls for camera
- Camera is auto-orbit only (no manual control)
- No microphone/live input support
- Single visualization mode
- No export/screenshot functionality

---

## 🔮 Future Enhancements

### Planned Features

- [ ] **Multi-format Support** - WAV, OGG, FLAC, AAC
- [ ] **User Camera Control** - Add OrbitControls for manual navigation
- [ ] **Microphone Input** - Live audio visualization
- [ ] **Settings Panel** - Grid size, colors, sensitivity adjustments
- [ ] **Visualization Modes** - Wave, circular, spiral patterns
- [ ] **Fullscreen Mode** - Immersive viewing experience
- [ ] **Export Functionality** - Save as video/GIF
- [ ] **Playlist Queue** - Multiple track support
- [ ] **Particle Effects** - Additional visual layers
- [ ] **Post-processing** - Bloom, glitch, chromatic aberration effects

### Enhancement Ideas

- VR/AR support for immersive experiences
- MIDI controller integration
- Audio effects (reverb, delay, filters)
- Custom color theme editor
- Social sharing capabilities
- Beat detection and tempo-synced effects

---

## 🤝 Contributing

Contributions are welcome! Here's how you can help:

1. **Fork the repository**
2. **Create a feature branch** (`git checkout -b feature/amazing-feature`)
3. **Commit your changes** (`git commit -m 'Add amazing feature'`)
4. **Push to the branch** (`git push origin feature/amazing-feature`)
5. **Open a Pull Request**

### Contribution Guidelines

- Maintain the single-file architecture
- Preserve the retro-futuristic aesthetic
- Keep code performant (60fps target)
- Add clear comments for complex logic
- Test thoroughly across browsers
- Update README for new features

---

## 📄 License

This project is open source and available under the [MIT License](LICENSE).

---

## 👤 Author

**Akshaj Molukutla**

- GitHub: [@a-k-2-k](https://github.com/a-k-2-k)
- Project: [VoxelFlow](https://github.com/a-k-2-k/voxelflow)

---

## 🙏 Acknowledgments

- **Three.js** - Amazing 3D library for the web
- **Web Audio API** - Powerful browser audio processing
- **Synthwave/Retrowave Community** - Design inspiration
- **Open Source Community** - For making projects like this possible

---

## 📚 Resources

### Learning Resources
- [Three.js Documentation](https://threejs.org/docs/)
- [Web Audio API Guide](https://developer.mozilla.org/en-US/docs/Web/API/Web_Audio_API)
- [WebGL Fundamentals](https://webglfundamentals.org/)

### Similar Projects
- [Waveform Playlist](https://github.com/naomiaro/waveform-playlist)
- [Audio Visualizer](https://github.com/willianjusten/awesome-audio-visualization)
- [Butterchurn](https://github.com/jberg/butterchurn) - Milkdrop visualizer

---

<div align="center">

**Made with 🎵 by [Akshaj Molukutla](https://github.com/a-k-2-k)**

⭐ Star this repo if you like VoxelFlow!

</div>

