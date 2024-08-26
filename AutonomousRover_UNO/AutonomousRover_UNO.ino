#include <SoftwareSerial.h>
#include <Arduino.h>
#include <ArduinoQueue.h>
#include <math.h>
#include <AFMotor.h>

SoftwareSerial esp32Serial(2, 4); // RX, TX (connect ESP32 TX2 to Pin 8, ESP32 RX2 to Pin 9)

bool isDataEverRead = false;
char dataFromESP32[32];
int readCount = 0;
int distanceToObstacle = 0;
float latitude = 0.0;
float longitude = 0.0;
int heading = 0;

struct Waypoint
{
  float deptLati;
  float deptLongi;
  int connections[3];
};

Waypoint waypoints[10] = {
    {12.902509, 77.517868, {1, -1, -1}}, // WP0
    {12.902462, 77.518135, {0, 2, 3}},   // WP1
    {12.902112, 77.518127, {1, 4, -1}},  // WP2
    {12.902403, 77.518631, {1, -1, -1}}, // WP3
    {12.902277, 77.519218, {2, 5, -1}},  // WP4
    {12.902328, 77.519165, {4, 6, -1}},  // WP5 coords not updated from here 
    {12.977598, 77.600566, {5, 7, -1}},  // WP6
    {12.978598, 77.601566, {6, 8, -1}},  // WP7
    {12.979598, 77.602566, {7, 9, -1}},  // WP8
    {12.980598, 77.603566, {8, -1, -1}}  // WP9
};

int startingWaypoint = 0;    // Start at waypoint 0 (WP0)
int destinationWaypoint = 1; // Destination at waypoint 9 (WP9)

ArduinoQueue<int> correctPath;

AF_DCMotor frontLeftMotor(1);  // Left Front
AF_DCMotor frontRightMotor(2); // Right Front
AF_DCMotor backLeftMotor(3);   // Left Rear
AF_DCMotor backRightMotor(4);  // Right Rear

void setup()
{
  esp32Serial.begin(9600);
  Serial.begin(9600);

  findPathFromStartToDestination();
}

void loop()
{
  if (isDataEverRead)
  {
    if (isDestinationNodeReached())
    {
      Serial.print("Destination reached");
      exit(0);
    }

    if (isOutsideAWayPoint())
    {
      Serial.print("Detected to be outside any waypoint. CurrLattitude: ");
      Serial.print(latitude);
      Serial.print(" CurrLongititude: ");
      Serial.println(longitude);

      navigateToNearestWayPoint();
    }

    int nextWayPoint = getNextWaypoint();
    if (nextWayPoint == -1)
    {
      Serial.print("Unable to find path to destination node");
      exit(1);
    }

    Serial.print("Next waypoint is: ");
    Serial.println(nextWayPoint);
    navigateToAWayPoint(nextWayPoint);
    readData();
  }
  else
  {
    Serial.println("Data is never read");
    isDataEverRead = readData(60000);
  }
  //    if (isOutsideAWayPoint) {
  //        navigateToNearestWayPoint();
  //        findPathFromStartToDestination();
  //    }
  // Add delay if required
}

bool isDestinationNodeReached()
{
  return waypoints[destinationWaypoint].deptLati == latitude && waypoints[destinationWaypoint].deptLongi == longitude;
}

// To navigate to nearest waypoint for the first time
bool isOutsideAWayPoint()
{
  for (int i = 0; i < 10; i++)
  {
    if (latitude == waypoints[i].deptLati && longitude == waypoints[i].deptLongi)
    {
      return false;
    }
  }
  return true;
}

int findIndexOfNearestWayPoint()
{
  int nearestIndex = -1;
  float minDistance = INFINITY;

  for (int i = 0; i < 10; i++)
  {
    float distance = calculateDistanceInCm(latitude, longitude, waypoints[i].deptLati, waypoints[i].deptLongi);
    if (distance < minDistance)
    {
      minDistance = distance;
      nearestIndex = i;
    }
  }

  return nearestIndex;
}

void navigateToNearestWayPoint()
{
  int indexOfNearestWayPoint = findIndexOfNearestWayPoint();
  Serial.print("Nearest waypoint is: ");
  Serial.print(indexOfNearestWayPoint);
  startingWaypoint = indexOfNearestWayPoint;

  navigateToAWayPoint(indexOfNearestWayPoint);
}

// Find where to move
void findPathFromStartToDestination()
{
  while (!correctPath.isEmpty())
  {
    correctPath.dequeue();
  }
  ArduinoQueue<int> q(10);    // Queue to manage BFS
  ArduinoQueue<int> path(10); // Queue to store intermediate result
  int parent[10];
  bool visited[10];

  // Initialize visited and parent arrays
  for (int i = 0; i < 10; i++)
  {
    visited[i] = false;
    parent[i] = -1;
  }

  visited[startingWaypoint] = true;
  q.enqueue(startingWaypoint);

  // Enqueue all unvisited neighbors
  while (!q.isEmpty())
  {
    int current = q.dequeue();

    // Check if the destination is reached
    if (current == destinationWaypoint)
    {
      break;
    }

    // Enqueue all unvisited neighbors
    for (int i = 0; i < 3; i++)
    {
      int neighbor = waypoints[current].connections[i];
      if (neighbor != -1 && !visited[neighbor])
      {
        visited[neighbor] = true;
        parent[neighbor] = current;
        q.enqueue(neighbor);
      }
    }

    while (!path.isEmpty())
    {
      correctPath.enqueue(path.dequeue());
    }
    // Add a print statement for the items of the correctPath
  }
}

int getNextWaypoint()
{
  if (!correctPath.isEmpty())
  {
    return correctPath.dequeue();
  }
  else
  {
    return -1; // Indicate the end of the path
  }
}

// Reading the input data
bool readData()
{
  readData(10000);
}

bool readData(long timeOut)
{
  readCount++;
  Serial.print("ReadCount: ");
  Serial.print(readCount);
  Serial.print(" Trying to receive data from ESP32. Timeout is: ");
  Serial.println(timeOut);
  while (timeOut > 0)
  {
    if (esp32Serial.available())
    {
      Serial.print("Data is available from ESP32. Processing");
      readAMessageFromESP32();
      extractInformationFromMessage(dataFromESP32);
      Serial.println("Data read succesfully");
      return true;
    }
    else
    {
      Serial.print("Timeout is: ");
      Serial.println(timeOut);
      timeOut -= 1000;
      delay(1000);
    }
  }

  Serial.print("ReadCount: ");
  Serial.print(readCount);
  Serial.println(" No Data was received");
  Serial.println("");
  return false;
}

void readAMessageFromESP32()
{
  //  int total = 0;
  //  char c;
  //  while (c = esp32Serial.read())
  //  {
  //    Serial.print(c);
  //    dataFromESP32[total] = c;
  //    total++;
  //  }
  //
  //  dataFromESP32[total] = '\0';
  int len = esp32Serial.readBytes(dataFromESP32, sizeof(dataFromESP32) - 1);
  dataFromESP32[len] = '\0';
  Serial.print("Message Read: ");
  Serial.println(dataFromESP32);
}

void extractInformationFromMessage(char data[])
{
    int firstCommaIndex = -1;
    int secondCommaIndex = -1;
    int thirdCommaIndex = -1;

    // Find the first comma
    for (int i = 0; i < 32; i++)
    {
        if (data[i] == ',')
        {
            firstCommaIndex = i;
            break;
        }
    }

    // Find the second comma
    if (firstCommaIndex != -1)
    {
        for (int i = firstCommaIndex + 1; i < 32; i++)
        {
            if (data[i] == ',')
            {
                secondCommaIndex = i;
                break;
            }
        }
    }

    // Find the third comma
    if (secondCommaIndex != -1)
    {
        for (int i = secondCommaIndex + 1; i < 32; i++)
        {
            if (data[i] == ',')
            {
                thirdCommaIndex = i;
                break;
            }
        }
    }

    if (firstCommaIndex != -1 && secondCommaIndex != -1 && thirdCommaIndex != -1)
    {
        // Extract distance
        String distanceStr = String(data).substring(0, firstCommaIndex);
        distanceToObstacle = distanceStr.toInt();

        // Extract latitude
        String latitudeStr = String(data).substring(firstCommaIndex + 1, secondCommaIndex);
        latitude = latitudeStr.toFloat();

        // Extract longitude
        String longitudeStr = String(data).substring(secondCommaIndex + 1, thirdCommaIndex);
        longitude = longitudeStr.toFloat();

        // Extract heading
        String headingStr = String(data).substring(thirdCommaIndex + 1);
        heading = headingStr.toInt();

        // Print the values with 6 digits after the decimal point
        char latBuffer[13]; // Buffer to hold formatted latitude
        char lonBuffer[13]; // Buffer to hold formatted longitude

        dtostrf(latitude, 8, 6, latBuffer);
        dtostrf(longitude, 9, 6, lonBuffer);

        Serial.print("Distance: ");
        Serial.println(distanceToObstacle);
        Serial.print("Latitude: ");
        Serial.println(latBuffer);
        Serial.print("Longitude: ");
        Serial.println(lonBuffer);
        Serial.print("Heading: ");
        Serial.println(heading);
    }
}


// Base method for rotation and movement calculation
void navigateToAWayPoint(int index)
{
  Serial.print("Navigating to index: ");
  Serial.print(index);
  Serial.print("CurrentLatitude: ");
  Serial.print(latitude);
  Serial.print("CurrentLongitude: ");
  Serial.print(longitude);
  Serial.print("DestinationLatitude: ");
  Serial.print(waypoints[index].deptLati);
  Serial.print("DestinationLongitude: ");
  Serial.println(waypoints[index].deptLongi);

  double bearing = calculateBearing(index);
  Serial.print("Heading angle: ");
  Serial.print(heading);
  Serial.print(" Bearing angle: ");
  Serial.println(bearing);

  if (heading != bearing)
  {
    rotateUntilWeFaceDestinationWaypoint(bearing);
  }

  moveForwardUntilWayPointIsReached(index);
}

// Calculation related to moving forward to here
void moveForwardUntilWayPointIsReached(int index)
{
  double distanceToWayPoint = calculateDistanceInCm(latitude, longitude, waypoints[index].deptLati, waypoints[index].deptLongi);
  if (distanceToObstacle > distanceToWayPoint)
  {
    double durationToMoveForward = calculateDurationToMoveForwardInMs(distanceToWayPoint);
    moveRoverForward(durationToMoveForward);
    return;
  }
  // Go till obstucle
  double durationTillObstucle30cm = calculateDurationToMoveForwardInMs(distanceToObstacle - 30);
  moveRoverForward(durationTillObstucle30cm);

  // Avoid obstucle
  int distanceMovedParallelToPath = avoidObstucle();
  int distanceRemainingToWayPoint = distanceToWayPoint - (distanceToObstacle - 30) - distanceMovedParallelToPath;

  // Reminder of the journey
  double timeRemainingToReachWayPoint = calculateDurationToMoveForwardInMs(distanceRemainingToWayPoint);
  moveRoverForward(timeRemainingToReachWayPoint);
}

int avoidObstucle()
{
  Serial.print("To avoid obstucle, Turning to: ");
  Serial.println((heading + 90) % 360);
  rotateRoverClockWise(calculateDurationToRotate((heading + 90) % 360));
  Serial.print("To avoid obstucle, moving 50 cm");
  moveRoverForward(calculateDurationToMoveForwardInMs(50));

  Serial.print("To avoid obstucle, Turning to: ");
  Serial.println((heading + 180) % 360);
  rotateRoverClockWise(calculateDurationToRotate((heading + 180) % 360));
  Serial.print("To avoid obstucle, moving 150 cm");
  moveRoverForward(calculateDurationToMoveForwardInMs(150));

  Serial.print("To avoid obstucle, Turning to: ");
  Serial.println((heading + 270) % 360);
  rotateRoverClockWise(calculateDurationToRotate((heading + 270) % 360));
  Serial.print("To avoid obstucle, moving 50 cm");
  moveRoverForward(calculateDurationToMoveForwardInMs(50));

  Serial.print("Obstucle avoided. Turning to: ");
  Serial.println(heading);
  rotateRoverClockWise(calculateDurationToRotate(heading));
  return 150;
}

double calculateDurationToMoveForwardInMs(double distance)
{
  double diameter = 6.7;

  // TODO: Update proper RPM
  double rpm = 100.0;

  double rps = rpm / 60.0;

  // Calculate the circumference of the wheel
  double circumference = M_PI * diameter;

  // Calculate the number of rotations needed
  double rotations = distance / circumference;

  // Calculate the time required for the rotations
  double time = rotations / rps;

  return time * 1000;
}

// Function to convert degrees to radians
double degreesToRadians(double degrees)
{
  return degrees * PI / 180.0;
}

float calculateDistanceInCm(float iLat1, float iLon1, float iLat2, float iLon2)
{
  float lat1 = degreesToRadians(iLat1);
  float lon1 = degreesToRadians(iLon1);
  float lat2 = degreesToRadians(iLat2);
  float lon2 = degreesToRadians(iLon2);

  // Haversine formula
  double dlat = lat2 - lat1;
  double dlon = lon2 - lon1;
  double a = sin(dlat / 2) * sin(dlat / 2) +
             cos(lat1) * cos(lat2) *
                 sin(dlon / 2) * sin(dlon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  // Distance in kilometers
  double distance = 6400 * c;

  return distance * 100000;
}

// Calculation related to rotating to face the direction
void rotateUntilWeFaceDestinationWaypoint(int bearing)
{
  double durationToRotate = calculateDurationToRotate(bearing);
  rotateRoverClockWise(durationToRotate);
}

int calculateBearing(int index)
{
  // Convert latitude and longitude from degrees to radians
  double lat1Rad = radians(latitude);
  double lon1Rad = radians(longitude);
  double lat2Rad = radians(waypoints[index].deptLati);
  double lon2Rad = radians(waypoints[index].deptLongi);

  // Compute the difference in longitudes
  double deltaLon = lon2Rad - lon1Rad;

  // Calculate the bearing
  double x = sin(deltaLon) * cos(lat2Rad);
  double y = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(deltaLon);
  double initialBearing = atan2(x, y);

  // Convert bearing from radians to degrees
  double initialBearingDeg = degrees(initialBearing);

  // Normalize the bearing to be within the range 0-360 degrees
  double bearing = fmod((initialBearingDeg + 360), 360);

  // Adjust so that 0 degrees points West
  bearing = (bearing + 270) - 360 * floor((bearing + 270) / 360);

  return bearing;
}

double calculateDurationToRotate(int bearing)
{
  // TODO: Update proper time
  double timeInMilisecondsFor360DegreeRotation = 8000;

  int angleOfRotation;
  if (heading < bearing)
  {
    angleOfRotation = bearing - heading;
  }
  else
  {
    angleOfRotation = 360 - (heading - bearing);
  }

  return 360 / timeInMilisecondsFor360DegreeRotation * angleOfRotation;
}

// Hardware code related to movement
void rotateRoverClockWise(double duration)
{
  Serial.print("Rotating clockwise for ");
  Serial.print(duration);
  Serial.println(" milleSeconds");

  // Right wheels backward
  frontRightMotor.setSpeed(255);
  frontRightMotor.run(BACKWARD);
  backRightMotor.setSpeed(255);
  backRightMotor.run(BACKWARD);

  // Left wheels forward
  frontLeftMotor.setSpeed(255);
  frontLeftMotor.run(FORWARD);
  backLeftMotor.setSpeed(255);
  backLeftMotor.run(FORWARD);

  delay(duration); // Rotate for the specified duration

  // Stop all motors
  stopMotors();
}
//
void moveRoverForward(double duration
)
{
  Serial.print("Moving rover forward for ");
  Serial.print(duration);
  Serial.println(" milleSeconds");

  // Right wheels backward
  frontRightMotor.setSpeed(255);
  frontRightMotor.run(FORWARD);
  backRightMotor.setSpeed(255);
  backRightMotor.run(FORWARD);
  // Left wheels forward
  frontLeftMotor.setSpeed(255);
  frontLeftMotor.run(FORWARD);
  backLeftMotor.setSpeed(255);
  backLeftMotor.run(FORWARD);
  
  delay(duration); // Rotate for the specified duration

  // Stop all motors
  stopMotors();
}
//
void stopMotors()
{
  frontLeftMotor.setSpeed(0);
  frontLeftMotor.run(RELEASE);
  frontRightMotor.setSpeed(0);
  frontRightMotor.run(RELEASE);
  backLeftMotor.setSpeed(0);
  backLeftMotor.run(RELEASE);
  backRightMotor.setSpeed(0);
  backRightMotor.run(RELEASE);
}
