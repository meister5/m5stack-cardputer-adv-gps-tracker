#include <M5Cardputer.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>
#include <vector>

// ===== Configuration =====
// M5Stack LoRa Cap uses the following pins for the GPS module:
#define GPS_RX_PIN 15     // Connects to GPS TX (GPS Module TX -> ESP32 RX)
#define GPS_TX_PIN 13     // Connects to GPS RX (GPS Module RX <- ESP32 TX)

// Using a fixed baud rate as requested
#define GPS_FIXED_BAUD 115200 

#define UPDATE_MS 500   // Refresh screen every 500ms
#define MIN_MOVE_M 2.0
#define MAX_POINTS 2000

// ===== GPS =====
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// ===== Data Structures =====
struct Point {
    double lat, lon, alt;
    float x, y;
    uint32_t time;
};

struct State {
    uint8_t mode;   // 0=menu, 1=record, 2=backtrack, 3=info, 4=debug
    uint8_t prevMode; // To detect mode changes
    bool recording;
    bool sdOk;
    bool gpsFix;
    uint32_t lastUpdate;
    uint32_t lastKey;
    uint32_t lastGPS;
    uint32_t gpsChars;
    uint32_t gpsSentences;
    float totalDist;
    String status;
    uint32_t statusTime;
    uint32_t currentBaud; // Holds the fixed baud rate
};

// ===== Globals =====
// Initialize prevMode to 255 to force initial redraw
State st = {4, 255, false, false, false, 0, 0, 0, 0, 0, 0.0, "", 0, GPS_FIXED_BAUD};
std::vector<Point> trail;
Point origin, current;
bool hasOrigin = false;
const double R = 6371000.0;
bool setupComplete = false;

// Function Prototypes (forward declarations)
void drawMap();
void drawBacktrackMap(float& minX, float& maxX, float& minY, float& maxY);
void drawDirectionalArrow(int w, int h, float scale, float minX, float minY, float bearing);
void setMsg(const char* m);
float dist(const Point& a, const Point& b);
float calcBearing(const Point& from, const Point& to);
void toXY(Point& p);
void handleKeys();
void drawStaticUI();
void updateDynamicUI();
void doRecord();
void showInfo();
void doBacktrack();
void saveTrack();


// ===== Setup =====
void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg);
    
    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.fillScreen(BLACK);
    M5Cardputer.Display.setTextColor(WHITE, BLACK); 
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setCursor(10, 10);
    M5Cardputer.Display.println("GPS TRACKER v2.1 (115200 Fixed)");
    
    M5Cardputer.Display.printf("GPS Pins: RX=%d TX=%d", GPS_RX_PIN, GPS_TX_PIN);
    
    // --- Start GPS at Fixed Baud Rate ---
    st.currentBaud = GPS_FIXED_BAUD;
    gpsSerial.begin(st.currentBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    M5Cardputer.Display.printf("\nGPS Serial: Active @ %u Baud", st.currentBaud);
    
    // Try SD
    M5Cardputer.Display.println("\nChecking SD...");
    st.sdOk = SD.begin();
    M5Cardputer.Display.println(st.sdOk ? "SD Card: OK" : "SD Card: Not found");
    
    M5Cardputer.Display.println("\nPress ENTER to start...");
    
    setupComplete = true;
    st.mode = 4; // Start in debug
}

// ===== Loop =====
void loop() {
    if (!setupComplete) return;
    
    uint32_t now = millis();
    M5Cardputer.update();
    
    // Feed GPS - CRITICAL: Must happen fast, outside of display delays
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        st.gpsChars++;
        
        // If in Debug mode, print raw characters to see what's coming in
        if (st.mode == 4 && st.prevMode == 4 && now - st.lastUpdate < UPDATE_MS) {
            // Clear area first
            M5Cardputer.Display.fillRect(5, 100, 230, 10, BLACK);
            M5Cardputer.Display.setCursor(5, 100);
            M5Cardputer.Display.print(c);
        }
        
        if (gps.encode(c)) {
            st.gpsSentences++;
            st.lastGPS = now;
        }
    }
    
    // Update GPS status logic
    if (gps.location.isValid()) {
        st.gpsFix = true;
        current.lat = gps.location.lat();
        current.lon = gps.location.lng();
        current.alt = gps.altitude.meters();
        current.time = now;
        if (hasOrigin) toXY(current);
        
        // --- Recording Logic ---
        if (st.recording && st.gpsFix && gps.location.age() < 1000) {
            if (!hasOrigin) {
                // Set origin and first point
                origin = current;
                origin.x = 0; origin.y = 0;
                hasOrigin = true;
                trail.push_back(origin);
                setMsg("Trail Started");
            } else {
                // Check distance from last point
                if (dist(current, trail.back()) > MIN_MOVE_M) {
                    if (trail.size() < MAX_POINTS) {
                        st.totalDist += dist(current, trail.back());
                        trail.push_back(current);
                    } else {
                        setMsg("Max Points!");
                    }
                }
            }
        }
        // --- End Recording Logic ---

    } else {
        if (now - st.lastGPS > 5000) st.gpsFix = false;
    }
    
    // Handle Keys
    if (M5Cardputer.Keyboard.isChange()) {
        handleKeys();
    }
    
    // Update Display
    bool modeChanged = (st.mode != st.prevMode);
    
    if (modeChanged) {
        M5Cardputer.Display.fillScreen(BLACK); 
        drawStaticUI(); 
        st.prevMode = st.mode;
        st.lastUpdate = now;
        updateDynamicUI();
    } 
    else if (now - st.lastUpdate > UPDATE_MS) {
        updateDynamicUI(); 
        st.lastUpdate = now;
    }
    
    // Clear status message overlay
    if (st.status.length() > 0 && now - st.statusTime > 3000) {
        M5Cardputer.Display.fillRect(50, 100, 150, 10, BLACK); // Clear status area
        st.status = "";
        if (st.mode == 0) { 
             updateDynamicUI();
        }
    }
    
    delay(1); // Small delay for stability
}

// ===== Keys =====
void handleKeys() {
    if (M5Cardputer.Keyboard.isPressed()) {
        Keyboard_Class::KeysState k = M5Cardputer.Keyboard.keysState();
        
        if (st.mode == 0) { // Menu
            if (k.word[0] == '1') {
                st.mode = 1; st.recording = true; st.totalDist = 0;
                if (trail.empty()) { hasOrigin = false; }
                setMsg("Recording...");
            }
            else if (k.word[0] == '2' && !trail.empty()) saveTrack();
            else if (k.word[0] == '3' && trail.size() > 1) st.mode = 2;
            else if (k.word[0] == '4') st.mode = 3;
            else if (k.word[0] == '5' && !trail.empty()) {
                trail.clear(); hasOrigin = false; st.totalDist = 0; setMsg("Cleared");
            }
            else if (k.word[0] == '9') st.mode = 4;
        }
        else {
            // In sub-modes, use ENTER to go back
            if (k.enter) {
                if (st.mode == 1) { st.recording = false; setMsg("Stopped"); }
                st.mode = 0;
            }
        }
    }
}

// ===== Display Logic (Static & Dynamic) =====

void drawStaticUI() {
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextColor(YELLOW, BLACK);
    
    switch(st.mode) {
        case 0: // Menu
            M5Cardputer.Display.setTextSize(2);
            M5Cardputer.Display.setCursor(40, 5); M5Cardputer.Display.print("GPS TRACKER");
            M5Cardputer.Display.setTextSize(1);
            M5Cardputer.Display.setTextColor(WHITE, BLACK);
            M5Cardputer.Display.setCursor(10, 35); M5Cardputer.Display.print("[1] Start Recording");
            M5Cardputer.Display.setCursor(10, 50); M5Cardputer.Display.print("[2] Save Track");
            M5Cardputer.Display.setCursor(10, 65); M5Cardputer.Display.print("[3] Backtrack (Map)");
            M5Cardputer.Display.setCursor(10, 80); M5Cardputer.Display.print("[4] GPS Info");
            M5Cardputer.Display.setCursor(10, 95); M5Cardputer.Display.print("[5] Clear Trail");
            M5Cardputer.Display.setCursor(10, 110); M5Cardputer.Display.print("[9] GPS Debug");
            break;
            
        case 1: // Record (Map is dynamic, minimal static UI)
            break;
            
        case 4: // Debug
            M5Cardputer.Display.setCursor(5, 5); M5Cardputer.Display.print("=== GPS DEBUG ===");
            M5Cardputer.Display.setTextColor(WHITE, BLACK);
            M5Cardputer.Display.setCursor(5, 25); M5Cardputer.Display.print("Baud Rate:");
            M5Cardputer.Display.setCursor(5, 40); M5Cardputer.Display.print("Chars RX:");
            M5Cardputer.Display.setCursor(5, 55); M5Cardputer.Display.print("Sentences:");
            M5Cardputer.Display.setCursor(5, 70); M5Cardputer.Display.print("Signal/Sats:");
            M5Cardputer.Display.setCursor(5, 85); M5Cardputer.Display.print("Lat/Lon:");
            M5Cardputer.Display.setTextColor(GREEN, BLACK);
            M5Cardputer.Display.setCursor(5, 115); M5Cardputer.Display.print("Raw data below. Press ENTER.");
            break;
            
        case 3: // Info
             M5Cardputer.Display.setCursor(5, 5); M5Cardputer.Display.print("=== GPS INFO ===");
             break;
    }
}

void updateDynamicUI() {
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextColor(WHITE, BLACK); 
    
    switch(st.mode) {
        case 0: // Menu Status
            M5Cardputer.Display.setCursor(5, 120);
            M5Cardputer.Display.setTextColor(st.gpsFix ? GREEN : RED, BLACK);
            M5Cardputer.Display.print(st.gpsFix ? "GPS: FIX " : "GPS: WAIT");
            
            M5Cardputer.Display.setCursor(180, 120);
            M5Cardputer.Display.setTextColor(st.sdOk ? GREEN : RED, BLACK);
            M5Cardputer.Display.print(st.sdOk ? "SD: OK " : "SD: NO ");
            
            if (st.status.length() > 0) {
                M5Cardputer.Display.setTextColor(YELLOW, BLACK);
                M5Cardputer.Display.setCursor(60, 105);
                M5Cardputer.Display.print(st.status);
            }
            break;
            
        case 1: // Recording (Map)
            doRecord(); 
            break;
            
        case 2: // Backtrack (Map with Arrow)
            doBacktrack();
            break;

        case 3: // Info
            showInfo();
            break;
            
        case 4: // Debug Values
            M5Cardputer.Display.setCursor(70, 25);
            M5Cardputer.Display.printf("%-8u", st.currentBaud);
            
            M5Cardputer.Display.setCursor(70, 40); 
            M5Cardputer.Display.printf("%-8u", st.gpsChars); 
            
            M5Cardputer.Display.setCursor(70, 55);
            M5Cardputer.Display.printf("%-8u", st.gpsSentences);
            
            M5Cardputer.Display.setCursor(110, 70);
            M5Cardputer.Display.print(st.gpsFix ? "FIXED   " : "SEARCHING");
            M5Cardputer.Display.printf("/%d   ", gps.satellites.value());
            
            M5Cardputer.Display.setCursor(70, 85);
            if(st.gpsFix) {
                M5Cardputer.Display.printf("%.5f, %.5f", gps.location.lat(), gps.location.lng());
            } else {
                M5Cardputer.Display.print("Waiting...            ");
            }
            break;
    }
}

// ===== Functions for Screens =====

void doRecord() {
    M5Cardputer.Display.fillScreen(BLACK);
    drawMap();
    
    // REC Indicator
    M5Cardputer.Display.fillCircle(230, 10, 5, (millis()/500)%2 == 0 ? RED : MAROON);
    
    M5Cardputer.Display.setTextColor(WHITE, BLACK);
    M5Cardputer.Display.setCursor(5, 95);
    M5Cardputer.Display.printf("Pts:%d Dist:%.0fm", trail.size(), st.totalDist);
    M5Cardputer.Display.setCursor(5, 110);
    M5Cardputer.Display.printf("Sat:%d Spd:%.1fm/s", gps.satellites.value(), gps.speed.mps());
}

void showInfo() {
    int y = 20;
    M5Cardputer.Display.setCursor(5, y); y+=15;
    M5Cardputer.Display.printf("Sats: %d  HDOP: %.2f  ", gps.satellites.value(), gps.hdop.hdop());
    M5Cardputer.Display.setCursor(5, y); y+=15;
    M5Cardputer.Display.printf("Alt: %.1fm  ", gps.altitude.meters());
    M5Cardputer.Display.setCursor(5, y); y+=15;
    M5Cardputer.Display.printf("Spd: %.1f km/h  ", gps.speed.kmph());
    M5Cardputer.Display.setCursor(5, y); y+=15;
    M5Cardputer.Display.printf("Date: %02d/%02d/%04d", gps.date.day(), gps.date.month(), gps.date.year());
    M5Cardputer.Display.setCursor(5, y); y+=15;
    M5Cardputer.Display.printf("Time: %02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
}

void doBacktrack() {
    M5Cardputer.Display.fillScreen(BLACK);
    
    if (!st.gpsFix || trail.size() < 2) {
        M5Cardputer.Display.setCursor(80, 50);
        M5Cardputer.Display.print("No Data / No Fix");
        return;
    }
    
    // 1. Find the target point (for Distance Display)
    float minD = 999999;
    int closest = 0;
    for (size_t i = 0; i < trail.size(); i++) {
        float d = dist(current, trail[i]);
        if (d < minD) { minD = d; closest = i; }
    }
    int target = (closest > 0 && minD < 15) ? closest - 1 : closest;
    float distance = dist(current, trail[target]);

    // 2. Determine the Arrow Direction (Current Heading)
    float currentHeading;
    if (gps.course.isValid() && gps.course.age() < 2000) {
        currentHeading = gps.course.deg(); // GPS course (direction of travel)
    } else {
        // Fallback: If course is not reliable, point to the next target
        currentHeading = calcBearing(current, trail[target]); 
    }

    // 3. Draw the map and get the scale/boundary parameters
    float minX, maxX, minY, maxY;
    drawBacktrackMap(minX, maxX, minY, maxY);

    // 4. Recalculate scale (needed for arrow positioning)
    int w = 240, h = 90; // Map area dimensions
    float rX = maxX - minX; float rY = maxY - minY;
    float scale = min(w/rX, h/rY);

    // 5. Draw the Directional Arrow (Red Triangle)
    drawDirectionalArrow(w, h, scale, minX, minY, currentHeading);
    
    // 6. Display distance and status below the map area
    M5Cardputer.Display.setTextColor(WHITE, BLACK);
    M5Cardputer.Display.setTextSize(2);
    M5Cardputer.Display.setCursor(10, 105);
    M5Cardputer.Display.printf("Dist to Target: %dm", (int)distance);
    
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setCursor(10, 120);
    M5Cardputer.Display.printf("Heading: %.0f deg | Target: #%d/%d", currentHeading, target, trail.size()-1);
}

// =========================================================================
// Helper Functions (Drawing)
// =========================================================================

void drawMap() {
    if (trail.empty()) {
        M5Cardputer.Display.setCursor(70, 60); M5Cardputer.Display.print("No Trail Data"); return;
    }
    // Auto-scale map logic
    float minX = trail[0].x, maxX = trail[0].x;
    float minY = trail[0].y, maxY = trail[0].y;
    for (auto& p : trail) {
        if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
    }
    if (st.gpsFix) {
        if (current.x < minX) minX = current.x; if (current.x > maxX) maxX = current.x;
        if (current.y < minY) minY = current.y; if (current.y > maxY) maxY = current.y;
    }
    
    float rX = maxX - minX; float rY = maxY - minY;
    if (rX < 10) rX = 10; if (rY < 10) rY = 10;
    
    // Padding
    minX -= rX*0.1; maxX += rX*0.1; minY -= rY*0.1; maxY += rY*0.1;
    rX = maxX - minX; rY = maxY - minY;
    
    int w = 240, h = 135; // Full screen size for recording map
    float scale = min(w/rX, h/rY);
    
    // Draw trail
    for (size_t i = 1; i < trail.size(); i++) {
        int x0 = (trail[i-1].x - minX) * scale;
        int y0 = h - (trail[i-1].y - minY) * scale; // Invert Y
        int x1 = (trail[i].x - minX) * scale;
        int y1 = h - (trail[i].y - minY) * scale;
        M5Cardputer.Display.drawLine(x0, y0, x1, y1, CYAN);
    }
    // Draw Current (simple dot for recording mode)
    if (st.gpsFix) {
        int cx = (current.x - minX) * scale;
        int cy = h - (current.y - minY) * scale;
        M5Cardputer.Display.fillCircle(cx, cy, 3, RED);
    }
}

void drawBacktrackMap(float& minX, float& maxX, float& minY, float& maxY) {
    if (trail.empty()) return;

    // Auto-scale map logic: Include all trail points and current position
    minX = trail[0].x; maxX = trail[0].x;
    minY = trail[0].y; maxY = trail[0].y;

    for (auto& p : trail) {
        if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
    }

    if (st.gpsFix) {
        if (current.x < minX) minX = current.x; if (current.x > maxX) maxX = current.x;
        if (current.y < minY) minY = current.y; if (current.y > maxY) maxY = current.y;
    }

    float rX = maxX - minX; float rY = maxY - minY;
    if (rX < 10) rX = 10; if (rY < 10) rY = 10;

    // Padding
    minX -= rX*0.1; maxX += rX*0.1; minY -= rY*0.1; maxY += rY*0.1;
    rX = maxX - minX; rY = maxY - minY;

    // Map area: 240x90 pixels
    int w = 240, h = 90; 
    float scale = min(w/rX, h/rY);

    // Draw trail
    for (size_t i = 1; i < trail.size(); i++) {
        int x0 = (trail[i-1].x - minX) * scale;
        int y0 = h - (trail[i-1].y - minY) * scale; // Invert Y
        int x1 = (trail[i].x - minX) * scale;
        int y1 = h - (trail[i].y - minY) * scale;
        
        x0 = constrain(x0, 0, w); y0 = constrain(y0, 0, h);
        x1 = constrain(x1, 0, w); y1 = constrain(y1, 0, h);

        M5Cardputer.Display.drawLine(x0, y0, x1, y1, CYAN);
    }
    
    // Draw Start and End Points
    int startX = (trail[0].x - minX) * scale;
    int startY = h - (trail[0].y - minY) * scale;
    int endX = (trail.back().x - minX) * scale;
    int endY = h - (trail.back().y - minY) * scale;
    
    M5Cardputer.Display.fillCircle(startX, startY, 4, GREEN); // Start
    M5Cardputer.Display.fillCircle(endX, endY, 4, YELLOW); // End
}


void drawDirectionalArrow(int w, int h, float scale, float minX, float minY, float bearing) {
    // Current position on the map
    int currentX = (current.x - minX) * scale;
    int currentY = h - (current.y - minY) * scale; // Y is inverted

    // Arrow parameters for a sharper triangle
    int arrowColor = RED;
    float lengthFactor = 6.0; // Distance of the tip/tail from center
    float widthFactor = 4.0;  // Half the width of the base

    
    // Convert bearing (0-360 deg) to radians for calculation
    float a = bearing * DEG_TO_RAD;

    // --- 1. Calculate the Tip (P0) ---
    // The tip is offset by lengthFactor from the center
    int p0x = currentX + (int)(lengthFactor * sin(a));
    int p0y = currentY - (int)(lengthFactor * cos(a)); 
    
    // --- 2. Calculate the Base Center ---
    // The base center is offset backward by a smaller amount (lengthFactor / 2)
    float back_offset_factor = lengthFactor / 2.0;
    int baseCenterX = currentX - (int)(back_offset_factor * sin(a));
    int baseCenterY = currentY + (int)(back_offset_factor * cos(a));
    
    // --- 3. Calculate the Base Corners (P1 and P2) ---
    // Calculate the perpendicular angle for the base
    float perp_a = a + (90.0 * DEG_TO_RAD); 
    
    // Base half-width offset
    int base_half_dx = (int)(widthFactor * sin(perp_a));
    int base_half_dy = -(int)(widthFactor * cos(perp_a));
    
    // Point 1 (Base corner 1)
    int p1x = baseCenterX + base_half_dx;
    int p1y = baseCenterY + base_half_dy;
    
    // Point 2 (Base corner 2)
    int p2x = baseCenterX - base_half_dx;
    int p2y = baseCenterY - base_half_dy;
    
    // Draw the triangle (arrowhead)
    M5Cardputer.Display.fillTriangle(p0x, p0y, p1x, p1y, p2x, p2y, arrowColor);
}

// ===== Helpers (Math, Save, Message) =====

void saveTrack() {
    if (!st.sdOk) { setMsg("No SD!"); return; }
    
    char name[32];
    snprintf(name, 32, "/trk_%lu.csv", millis()/1000); // Simple naming
    
    File f = SD.open(name, FILE_WRITE);
    if (!f) { setMsg("Write Fail!"); return; }
    
    f.println("time,lat,lon,alt");
    for (auto& p : trail) {
        f.printf("%u,%.8f,%.8f,%.2f\n", p.time, p.lat, p.lon, p.alt);
    }
    f.close();
    setMsg("Saved!");
}

void setMsg(const char* m) {
    st.status = String(m);
    st.statusTime = millis();
    if(st.mode == 0) updateDynamicUI(); // Force update to show msg in menu mode
}

// Math functions
float dist(const Point& a, const Point& b) {
    double lat1 = a.lat * DEG_TO_RAD;
    double lat2 = b.lat * DEG_TO_RAD;
    double dLat = (b.lat - a.lat) * DEG_TO_RAD;
    double dLon = (b.lon - a.lon) * DEG_TO_RAD;
    double x = sin(dLat/2) * sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
    return R * 2 * atan2(sqrt(x), sqrt(1-x));
}

float calcBearing(const Point& from, const Point& to) {
    double lat1 = from.lat * DEG_TO_RAD;
    double lat2 = to.lat * DEG_TO_RAD;
    double dLon = (to.lon - from.lon) * DEG_TO_RAD;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double b = atan2(y, x) * 180.0 / PI;
    return fmod(b + 360.0, 360.0);
}

void toXY(Point& p) {
    if (!hasOrigin) return;
    double dLat = (p.lat - origin.lat) * DEG_TO_RAD * R;
    double dLon = (p.lon - origin.lon) * DEG_TO_RAD * R * cos(origin.lat * DEG_TO_RAD);
    p.x = dLon; p.y = dLat;
}