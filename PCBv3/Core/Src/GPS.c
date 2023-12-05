
#include <stdio.h>
#include <math.h>
#include <gps.h>
#include <string.h>
#include <stdlib.h>

/*
 * Function to extract latitude from a NMEA GGA sentence
 */
double get_lat(char *gga)
{
	double latitude = 0.0;

	// Create a copy of the input string to avoid modifying the original
	char gga_cpy[128];
	strncpy(gga_cpy, gga, 128);

	// Use strtok to tokenize the copied string using ","
	char *token = strtok(gga_cpy, ",");

	// Iterate through the tokens
	while (token != NULL)
	{
		// Check if the token is "N" (North) or "S" (South)
		if ((strcmp(token, "N") == 0) || (strcmp(token, "S") == 0))
		{
			// Break the loop if "N" or "S" is found (latitude hemisphere indicator)
			break;
		}
		else
		{
			// Convert the token to a floating-point number
			latitude = atof(token);
		}
		// Move to the next token
		token = strtok(NULL, ",");
	}

		return latitude;
}

/*
 * Function to extract longitude from a NMEA GGA sentence
 */
double get_lon(char *gga)
{
	double longitude = 0.0;

	// Create a copy of the input string to avoid modifying the original
	char gga_cpy[128];
	strncpy(gga_cpy, gga, 128);

	// Use strtok to tokenize the copied string using ","
	char *token = strtok(gga_cpy, ",");

	// Iterate through the tokens
	while (token != NULL)
	{
		// Check if the token is "W" (West) or "E" (East)
		if ((strcmp(token, "W") == 0) || (strcmp(token, "E") == 0))
		{
			// Break the loop if "W" or "E" is found (longitude hemisphere indicator)
			break;
		}
		else
		{
			// Convert the token to a floating-point number
			longitude = atof(token);
		}
		// Move to the next token
		token = strtok(NULL, ",");
	}

		return longitude;
}

/*
 * Function to extract and format time information from a NMEA GGA sentence
 */
void get_time(char *gga, char *time)
{
	// Declare variables to store hours, minutes, and seconds
	int hours, minutes, seconds;

	// Find the first and second commas in the NMEA GGA sentence
	char *comma1 = strchr(gga, ',');
	char *comma2 = strchr(comma1 + 1, ',');

	// Calculate the length of the substring between the two commas
	size_t length = comma2 - (comma1 + 1);

	// Copy the substring between the commas to the 'time' variable
	strncpy(time, comma1 + 1, length);

	// Add a null terminator to the 'time' string
	time[length] = '\0';

	// Use sscanf to parse the time string and extract hours, minutes, and seconds
	sscanf(time, "%2d%2d%2d", &hours, &minutes, &seconds);

	// Adjust the hours to be EST from GMT
	hours -= 5;

	// Check if the adjusted hours are negative and correct them
	if (hours < 0) {
		hours += 24;
	}

	// Format the time as HH:MM:SS and store it back in the 'time' variable
	sprintf(time, "%02d:%02d:%02d\n", hours, minutes, seconds);
}

/*
 * Function to extract altitude in meters from GGA sentence
 */
double get_alt(char *gga)
{
	double altitude = 0.0;

	// Create a copy of the input string to avoid modifying the original
	char gga_cpy[128];
	strncpy(gga_cpy, gga, 128);

	// Use strtok to tokenize the copied string using ","
	char *token = strtok(gga_cpy, ",");

	// Iterate through the tokens
	while (token != NULL)
	{
		// Check if the token is "M" (this represents meters in GGA sentence)
		if (strcmp(token, "M") == 0)
		{
			break;
		}
		else
		{
			// Convert the token to a floating-point number
			altitude = atof(token);
		}

		// Move to the next token
		token = strtok(NULL, ",");
	}

		return altitude;
}

/*
 * Function to convert degrees and decimal minutes (DDM) to decimal degrees (DD)
 */
double ddm2dd(double ddm) {
	// Extract whole degrees from DDM format
    double degrees = floor(ddm / 100.0);

    // Extract minutes from DDM format
    double minutes = ddm - degrees * 100.0;

    // Calculate decimal degrees by combining whole degrees and minutes
    double dd = degrees + minutes / 60.0;

    return dd;
}

/*
 * Function to convert degrees to radians
 */
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

/*
 * Function to calculate the distance between two coordinates using Haversine formula
 */
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // Convert latitude and longitude from degrees to radians
    lat1 = degreesToRadians(ddm2dd(lat1));
    lon1 = degreesToRadians(ddm2dd(lon1));
    lat2 = degreesToRadians(ddm2dd(lat2));
    lon2 = degreesToRadians(ddm2dd(lon2));

    // Haversine formula
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = sin(dlat/2) * sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = RADIUS_OF_EARTH * c;

    return distance;
}
