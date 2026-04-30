#include <TinyGPS++.h>

// Choose a hardware serial port (Serial1, Serial2, etc.)
#define GPS_SERIAL Serial1

TinyGPSPlus gps;

#include <math.h>

#define DEG_TO_RAD 0.017453292519943295

void latLonToUTM_Zone43(double lat, double lon,
                        double *easting, double *northing)
{
    // WGS84 constants
    const double a  = 6378137.0;
    const double f  = 1 / 298.257223563;
    const double k0 = 0.9996;

    const double e2 = f * (2 - f);
    const double ep2 = e2 / (1 - e2);

    // Convert to radians
    double phi = lat * DEG_TO_RAD;
    double lambda = lon * DEG_TO_RAD;

    // Central meridian (Zone 43)
    double lambda0 = 75.0 * DEG_TO_RAD;

    double N = a / sqrt(1 - e2 * sin(phi) * sin(phi));
    double T = tan(phi) * tan(phi);
    double C = ep2 * cos(phi) * cos(phi);
    double A = cos(phi) * (lambda - lambda0);

    // Meridian arc
    double M = a * (
        (1 - e2/4 - 3*e2*e2/64 - 5*e2*e2*e2/256) * phi
        - (3*e2/8 + 3*e2*e2/32 + 45*e2*e2*e2/1024) * sin(2*phi)
        + (15*e2*e2/256 + 45*e2*e2*e2/1024) * sin(4*phi)
        - (35*e2*e2*e2/3072) * sin(6*phi)
    );

    // Easting
    *easting = k0 * N * (
        A + (1 - T + C) * pow(A,3)/6
        + (5 - 18*T + T*T + 72*C - 58*ep2) * pow(A,5)/120
    ) + 500000.0;

    // Northing (Northern hemisphere → no offset)
    *northing = k0 * (
        M + N * tan(phi) * (
            A*A/2
            + (5 - T + 9*C + 4*C*C) * pow(A,4)/24
            + (61 - 58*T + T*T + 600*C - 330*ep2) * pow(A,6)/720
        )
    );
}

void setup() {
  Serial.begin(115200);      // USB Serial for debugging
  GPS_SERIAL.begin(115200);  // M10 GPS default baud rate
  // Serial.println("time_ms,lat,lon,easting,northing,sat");
  Serial.println("latitude, longitude, saturation");
}

void loop() {
  // Read from GPS and feed to TinyGPS++
  // // Serial.println("Hello");
  while (GPS_SERIAL.available() > 0) {
    // Serial.println("Hello1");
    if (gps.encode(GPS_SERIAL.read())) {
      // Serial.println("Hello2");
      if (gps.location.isUpdated()) {
        // Serial.println("Hello3");
        // Serial.print("LAT="); 
        Serial.print(gps.location.lat(), 6);
        Serial.print(",");
        // Serial.print("LONG="); 
        Serial.print(gps.location.lng(), 6);
        Serial.print(",");
        // Serial.print("SAT=");  
        Serial.println(gps.satellites.value());


        // UTM COORDS -----------------------------
        // double E, N;
        // latLonToUTM_Zone43(
        //     gps.location.lat(),
        //     gps.location.lng(),
        //     &E, &N
        // );

        // Serial.print(millis()); Serial.print(",");
        // Serial.print(gps.location.lat(), 6); Serial.print(",");
        // Serial.print(gps.location.lng(), 6); Serial.print(",");
        // Serial.print(E, 2); Serial.print(",");
        // Serial.print(N, 2); Serial.print(",");
        // Serial.println(gps.satellites.value());
        // Serial.println("Hello4");
        delay(1000);
      }
    }
  }
  // while (GPS_SERIAL.available()) {
  //   char c = GPS_SERIAL.read();
  //   Serial.write(c);  // print raw data
  // }
//   if (gps.encode(GPS_SERIAL.read())) {
//   Serial.println("Sentence parsed");

//   Serial.print("Satellites: ");
//   Serial.println(gps.satellites.value());

//   if (gps.location.isValid()) {
//     Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
//     Serial.print("LON="); Serial.println(gps.location.lng(), 6);
//   } else {
//     Serial.println("No valid location yet");
//   }
// }
}
