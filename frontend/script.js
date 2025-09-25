class EEGBlinkController {
    constructor() {
        this.port = null;
        this.reader = null;
        this.isConnected = false;
        this.signalQuality = 0;
        this.currentThreshold = 0;
        this.rawSignal = 0;
        this.filteredSignal = 0;
        this.envelope = 0;
        this.baseline = 0;
        this.menuActive = false;
        this.currentFocusOption = 1;
        this.systemCalibrated = false;
        
        this.initializeUI();
        this.setupRealTimeUpdates();
    }

    initializeUI() {
        // Get UI elements
        this.connectBtn = document.getElementById('connectBtn');
        this.statusIndicator = document.getElementById('status');
        this.menuStatus = document.getElementById('menuStatus');
        this.blinkIndicator = document.getElementById('blinkIndicator');
        this.logsContainer = document.getElementById('logs');
        
        // Add signal quality display
        this.addSignalQualityDisplay();
        
        // Bind event handlers
        this.connectBtn.addEventListener('click', () => this.connectToDevice());
        
        this.addLog('🧠 BioAmp EEG System v2.0 initialized', 'info');
        this.addLog('📡 Ready to connect to Arduino with BioAmp', 'info');
    }
    
    addSignalQualityDisplay() {
        const menuStatus = document.querySelector('.menu-status');
        const signalQualityHTML = `
            <div class="signal-quality">
                <div class="signal-quality-label">Signal Quality</div>
                <div class="signal-quality-bar">
                    <div class="signal-quality-fill" id="signalQualityFill" style="width: 0%"></div>
                </div>
                <div class="signal-quality-value" id="signalQualityValue">0%</div>
            </div>
        `;
        menuStatus.insertAdjacentHTML('afterend', signalQualityHTML);
    }

    setupRealTimeUpdates() {
        // Update UI every 100ms
        setInterval(() => {
            this.updateSignalQualityDisplay();
            this.updateBlinkIndicator();
        }, 100);
    }

    updateSignalQualityDisplay() {
        const fill = document.getElementById('signalQualityFill');
        const value = document.getElementById('signalQualityValue');
        
        if (fill && value) {
            fill.style.width = `${this.signalQuality}%`;
            value.textContent = `${Math.round(this.signalQuality)}%`;
            
            // Color coding based on quality
            if (this.signalQuality >= 80) {
                fill.style.background = '#10b981'; // Green
            } else if (this.signalQuality >= 60) {
                fill.style.background = '#f59e0b'; // Yellow
            } else {
                fill.style.background = '#ef4444'; // Red
            }
        }
    }

    updateBlinkIndicator() {
        if (!this.systemCalibrated) {
            this.blinkIndicator.style.color = '#f59e0b';
            this.blinkIndicator.style.transform = 'scale(1)';
            return;
        }
        
        // Calculate signal strength relative to baseline
        const signalStrength = this.baseline > 0 ? (this.envelope - this.baseline) / this.baseline : 0;
        
        if (signalStrength > 0.3) {
            this.blinkIndicator.style.transform = 'scale(1.3)';
            this.blinkIndicator.style.color = '#10b981';
        } else if (signalStrength > 0.1) {
            this.blinkIndicator.style.transform = 'scale(1.1)';
            this.blinkIndicator.style.color = '#f59e0b';
        } else {
            this.blinkIndicator.style.transform = 'scale(1)';
            this.blinkIndicator.style.color = '#6b7280';
        }
    }

    async connectToDevice() {
        try {
            if (!('serial' in navigator)) {
                throw new Error('Web Serial API not supported. Please use Chrome or Edge browser.');
            }

            this.addLog('🔌 Requesting serial port connection...', 'info');
            this.connectBtn.disabled = true;
            this.connectBtn.classList.add('loading');
            this.connectBtn.textContent = 'Connecting...';

            // Request serial port
            this.port = await navigator.serial.requestPort();
            
            // Open with optimized settings for BioAmp
            await this.port.open({ 
                baudRate: 115200,
                dataBits: 8,
                stopBits: 1,
                parity: 'none',
                flowControl: 'none'
            });
            
            this.isConnected = true;
            this.updateConnectionStatus(true);
            this.addLog('✅ Successfully connected to BioAmp system!', 'success');
            
            // Start reading data
            this.startReading();

        } catch (error) {
            this.addLog(`❌ Connection failed: ${error.message}`, 'error');
            this.connectBtn.disabled = false;
            this.connectBtn.classList.remove('loading');
            this.connectBtn.innerHTML = '<span>⚡</span>Connect to EEG Device';
        }
    }

    async startReading() {
        const textDecoder = new TextDecoderStream();
        const readableStreamClosed = this.port.readable.pipeTo(textDecoder.writable);
        this.reader = textDecoder.readable.getReader();

        try {
            while (true) {
                const { value, done } = await this.reader.read();
                if (done) break;
                
                this.processSerialData(value);
            }
        } catch (error) {
            this.addLog(`📡 Serial communication error: ${error.message}`, 'error');
        } finally {
            this.reader.releaseLock();
        }
    }

    processSensorData(dataLine) {
        // Parse simplified format: "DATA:raw,filtered,envelope,threshold,baseline,quality,menu,focus,calibrated"
        if (dataLine.startsWith('DATA:')) {
            const values = dataLine.substring(5).split(',');
            if (values.length >= 9) {
                this.rawSignal = parseFloat(values[0]) || 0;
                this.filteredSignal = parseFloat(values[1]) || 0;
                this.envelope = parseFloat(values[2]) || 0;
                this.currentThreshold = parseFloat(values[3]) || 0;
                this.baseline = parseFloat(values[4]) || 0;
                this.signalQuality = parseFloat(values[5]) || 0;
                this.menuActive = values[6] === '1';
                this.currentFocusOption = parseInt(values[7]) || 1;
                this.systemCalibrated = values[8] === '1';
            }
        }
    }

    processSerialData(data) {
        const lines = data.split('\n');
        
        for (let line of lines) {
            line = line.trim();
            if (!line) continue;
            
            try {
                // Process different message types
                if (line.includes('CALIBRATION_STARTED')) {
                    this.startCalibration();
                } else if (line.includes('CALIBRATION_COMPLETE')) {
                    this.completeCalibration();
                } else if (line.startsWith('CALIBRATION_PROGRESS:')) {
                    const progress = parseInt(line.split(':')[1]);
                    this.updateCalibrationProgress(progress);
                } else if (line.includes('MENU_ACTIVATED')) {
                    this.activateMenu();
                } else if (line.includes('MENU_DEACTIVATED')) {
                    this.deactivateMenu();
                } else if (line.includes('MENU_TIMEOUT')) {
                    this.deactivateMenu();
                    this.addLog('⏰ Menu timeout', 'warning');
                } else if (line.startsWith('FOCUS_OPTION:')) {
                    const option = parseInt(line.split(':')[1]);
                    this.focusOption(option);
                } else if (line.startsWith('OPTION_SELECTED:')) {
                    const option = parseInt(line.split(':')[1]);
                    this.selectOption(option);
                } else if (line.includes('SINGLE_BLINK_FOCUS')) {
                    this.showBlinkFeedback('single');
                    this.addLog('👁️ Focus cycling', 'info');
                } else if (line.includes('DOUBLE_BLINK_SELECT')) {
                    this.showBlinkFeedback('double');
                    this.addLog('👁️👁️ Option selected!', 'success');
                } else if (line.startsWith('VALID_BLINK:')) {
                    this.addLog('✅ Valid blink detected', 'success');
                } else if (line.startsWith('INVALID_BLINK:')) {
                    this.addLog('❌ Invalid blink', 'warning');
                } else if (line.startsWith('DATA:')) {
                    this.processSensorData(line);
                } else if (line.startsWith('SIGNAL_QUALITY:')) {
                    this.signalQuality = parseFloat(line.split(':')[1]) || 0;
                }
            } catch (error) {
                // Ignore parsing errors to prevent crashes
                console.warn('Error parsing line:', line, error);
            }
        }
    }

    startCalibration() {
        this.addLog('🔧 System calibration started - Please remain still', 'info');
        this.menuStatus.textContent = 'Calibrating...';
        this.menuStatus.style.color = '#f59e0b';
        
        // Show calibration overlay
        this.showCalibrationOverlay();
    }

    updateCalibrationProgress(progress) {
        this.addLog(`📊 Calibration: ${progress + 1}/3 seconds`, 'info');
        
        const progressBar = document.getElementById('calibrationProgress');
        if (progressBar) {
            progressBar.style.width = `${((progress + 1) / 3) * 100}%`;
        }
    }

    completeCalibration() {
        this.systemCalibrated = true;
        this.addLog('✅ Calibration complete - System ready!', 'success');
        this.menuStatus.textContent = 'Ready - Blink to activate';
        this.menuStatus.style.color = '#10b981';
        
        // Hide calibration overlay
        this.hideCalibrationOverlay();
    }

    showCalibrationOverlay() {
        const overlay = document.createElement('div');
        overlay.id = 'calibrationOverlay';
        overlay.style.cssText = `
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.8);
            display: flex;
            justify-content: center;
            align-items: center;
            z-index: 10000;
        `;
        
        overlay.innerHTML = `
            <div style="background: white; padding: 2rem; border-radius: 16px; text-align: center; max-width: 400px;">
                <h2 style="color: #374151; margin-bottom: 1rem;">🔧 System Calibration</h2>
                <p style="color: #6b7280; margin-bottom: 1.5rem;">Please remain still for 3 seconds</p>
                <div style="background: #e5e7eb; height: 8px; border-radius: 4px; overflow: hidden;">
                    <div id="calibrationProgress" style="height: 100%; background: #10b981; width: 0%; transition: width 0.3s ease;"></div>
                </div>
                <p style="color: #6b7280; margin-top: 1rem; font-size: 0.9rem;">Establishing baseline...</p>
            </div>
        `;
        
        document.body.appendChild(overlay);
    }

    hideCalibrationOverlay() {
        const overlay = document.getElementById('calibrationOverlay');
        if (overlay) {
            document.body.removeChild(overlay);
        }
    }

    updateConnectionStatus(connected) {
        if (connected) {
            this.statusIndicator.className = 'status-indicator connected';
            this.statusIndicator.innerHTML = '<div class="status-dot"></div><span>Connected</span>';
            this.connectBtn.innerHTML = '<span>✅</span>Connected';
            this.connectBtn.disabled = true;
            this.connectBtn.classList.remove('loading');
            this.menuStatus.textContent = 'Initializing...';
        } else {
            this.statusIndicator.className = 'status-indicator disconnected';
            this.statusIndicator.innerHTML = '<div class="status-dot"></div><span>Disconnected</span>';
            this.connectBtn.innerHTML = '<span>⚡</span>Connect to EEG Device';
            this.connectBtn.disabled = false;
            this.connectBtn.classList.remove('loading');
            this.menuStatus.textContent = 'Inactive';
        }
    }

    addLog(message, type = 'info') {
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry ${type}`;
        logEntry.textContent = `[${timestamp}] ${message}`;
        
        this.logsContainer.appendChild(logEntry);
        this.logsContainer.scrollTop = this.logsContainer.scrollHeight;
        
        // Keep only last 50 log entries to save memory
        while (this.logsContainer.children.length > 50) {
            this.logsContainer.removeChild(this.logsContainer.firstChild);
        }
    }
}

// Initialize the controller when page loads
document.addEventListener('DOMContentLoaded', () => {
    if ('serial' in navigator) {
        new EEGBlinkController();
    } else {
        document.getElementById('logs').innerHTML = 
            '<div class="log-entry error">❌ Web Serial API not supported. Please use Chrome or Edge browser with HTTPS.</div>';
    }
});
