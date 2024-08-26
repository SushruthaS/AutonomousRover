#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Create GPS object
TinyGPSPlus gps;

// Define software serial pins for the GPS module
SoftwareSerial ss(4, 3); // RX, TX

void setup() {
  Serial.begin(9600);
  ss.begin(9600);
}

void loop() {
  // Retrieve GPS data
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }
  
  // Check if GPS has valid location data
  if (gps.location.isValid()) {
    double currentLat = gps.location.lat();
    double currentLng = gps.location.lng();
    
    // Define destination coordinates (example)
    double destLat = 37.7749; // Destination latitude
    double destLng = -122.4194; // Destination longitude
    
    // Calculate heading
    double heading = calculateHeading(currentLat, currentLng, destLat, destLng);
    Serial.print("Heading: ");
    Serial.println(heading);
  }
}

// Function to calculate heading
double calculateHeading(double lat1, double lon1, double lat2, double lon2) {
  double dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double bearing = atan2(y, x);
  
  // Convert bearing from radians to degrees
  bearing = degrees(bearing);
  
  // Normalize to 0-360 degrees
  bearing = fmod((bearing + 360), 360);
  
  return bearing;
}
