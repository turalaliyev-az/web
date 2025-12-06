# Robot Control Panel - Enhanced Version

## Overview

This is an enhanced version of the Robot Control Panel application that provides a web-based interface for controlling a robot with live video streaming. The application has been significantly improved with better error handling, security features, configuration management, and user interface enhancements.

## Features

### Core Functionality
- **Live Video Streaming**: Real-time video feed from camera with configurable resolution and quality
- **Robot Control**: Web-based control interface with keyboard and touch support
- **Multi-Platform Camera Support**: Automatic detection of cameras with fallback to demo mode
- **USB Serial Communication**: Automatic serial port detection and communication with robot hardware

### Serial Communication Features
- **Auto Port Detection**: Automatic detection of USB serial devices (Arduino, etc.)
- **Configurable Baud Rate**: Customizable serial communication parameters
- **JSON/Text Command Format**: Flexible command formatting options
- **Automatic Reconnection**: Self-healing serial connection with retry logic
- **Serial Status Monitoring**: Real-time serial connection status in UI
- **Background Processing**: Non-blocking serial communication thread

### Security Enhancements
- **Authentication System**: Login/logout functionality
- **CSRF Protection**: Cross-Site Request Forgery prevention
- **Session Management**: Secure session handling with timeout
- **Rate Limiting**: Command rate limiting to prevent abuse
- **Secure Cookies**: HTTP-only, Secure, SameSite cookie settings

### Error Handling & Reliability
- **Comprehensive Error Handling**: Graceful handling of camera, network, and serial errors
- **Automatic Recovery**: Self-healing mechanisms for common failure scenarios
- **Detailed Logging**: File and console logging with timestamps
- **Resource Management**: Proper cleanup of camera, serial, and system resources

### Configuration Management
- **JSON Configuration**: External configuration file for easy customization
- **Environment Adaptation**: Automatic fallback to default settings
- **Runtime Configuration**: Dynamic adjustment of video and serial parameters
- **Serial Configuration**: Full control over serial communication settings

### User Interface Improvements
- **Responsive Design**: Mobile-friendly interface
- **Visual Feedback**: Connection status indicators and real-time FPS display
- **Enhanced Controls**: Improved button layout and keyboard support
- **Status Monitoring**: Visual connection status and command feedback
- **System Dashboard**: Comprehensive status display for video, network, and serial systems

### Performance Optimizations
- **FPS Monitoring**: Real-time frame rate display
- **Adaptive Quality**: Configurable JPEG compression
- **Efficient Resource Usage**: Optimized camera parameter handling
- **Non-blocking Operations**: Background threads for serial communication

## Installation

### Prerequisites
- Python 3.7+
- OpenCV (cv2)
- Flask
- Flask-SocketIO
- NumPy
- PySerial (for USB serial communication)

### Setup
```bash
pip install opencv-python flask flask-socketio numpy pyserial
```

### Optional Dependencies
For full serial communication functionality:
```bash
pip install pyserial
```

Note: If `pyserial` is not installed, the application will run in a degraded mode where serial communication is disabled but all other features remain functional.

## Configuration

Edit `config.json` to customize the application:

```json
{
    "SECRET_KEY": "your_secret_key_here",
    "HOST": "0.0.0.0",
    "PORT": 9966,
    "VIDEO_WIDTH": 320,
    "VIDEO_HEIGHT": 240,
    "VIDEO_FPS": 30,
    "JPEG_QUALITY": 55,
    "CORS_ALLOWED_ORIGINS": "*",
    "PING_INTERVAL": 0.5,
    "PING_TIMEOUT": 10,
    "MAX_COMMAND_RATE": 100,
    "SESSION_TIMEOUT": 3600
}
```

#### Serial Configuration Options

- **SERIAL_ENABLED**: Enable/disable serial communication (true/false)
- **SERIAL_BAUDRATE**: Communication speed (9600, 19200, 38400, 57600, 115200)
- **SERIAL_TIMEOUT**: Read/write timeout in seconds
- **SERIAL_PORT**: Specific port to use (e.g., "COM3", "/dev/ttyUSB0"), null for auto-detect
- **SERIAL_AUTO_DETECT**: Automatically detect Arduino/USB serial devices
- **SERIAL_COMMAND_FORMAT**: "json" for structured data or "text" for simple commands
- **SERIAL_RECONNECT_DELAY**: Delay between reconnection attempts in seconds

## Usage

### Starting the Server
```bash
python app.py
```

### Accessing the Interface
1. Open a web browser and navigate to `http://localhost:9966`
2. Log in with any credentials (demo mode)
3. Use the on-screen controls or keyboard (WASD/Arrow keys) to control the robot
4. Monitor the live video feed and status information

### Keyboard Controls
- **W / ↑**: Move Forward
- **S / ↓**: Move Backward
- **A / ←**: Turn Left
- **D / →**: Turn Right
- **Space**: Stop

## Security Notes

- The application uses session-based authentication
- CSRF tokens are automatically generated and validated
- All sensitive routes require authentication
- Command rate limiting prevents abuse
- Secure cookie settings are enforced

## Troubleshooting

### Common Issues

**Camera not detected:**
- Check camera connections
- Verify camera permissions
- The application will fall back to demo mode automatically

**Connection issues:**
- Check network connectivity
- Verify the server is running
- Refresh the browser page

**Performance problems:**
- Reduce video resolution in config.json
- Lower JPEG quality setting
- Check system resource usage

**Serial communication issues:**
- Verify USB cable connection
- Check if the device appears in `ls /dev/tty*` (Linux) or Device Manager (Windows)
- Ensure correct baud rate matches both ends
- Check user permissions for serial port access
- Verify the Arduino/robot device is powered on
- Check `robot_control.log` for serial detection errors

## Development

### Project Structure
```
.
├── app.py                  # Main application
├── config.json             # Configuration file
├── templates/
│   └── index.html           # Web interface
├── robot_control.log        # Log file (auto-generated)
└── README.md                # This file
```

### Extending Functionality

To add new robot commands:
1. Add the command to the `valid_commands` list in `handle_robot_command`
2. Update the UI to include the new command button
3. Implement the actual robot control logic

## License

This project is provided as-is for educational and demonstration purposes.

## Support

For issues or questions, please refer to the comprehensive logging system and error messages provided by the application.