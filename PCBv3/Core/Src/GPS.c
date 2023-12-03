/*
 * GPS.c
 *
 *  Created on: Oct 22, 2023
 *      Author: martinguarnieri + Isaac Violette
 */

#include <stdio.h>
#include <math.h>
#include <gps.h>
#include <string.h>
#include <stdlib.h>

double get_lat(char *gga)
{
	double latitude = 0.0;

	char gga_cpy[256];
	strncpy(gga_cpy, gga, 256);

	char *token = strtok(gga_cpy, ",");

	while (token != NULL)
	{
		if ((strcmp(token, "N") == 0) || (strcmp(token, "S") == 0))
		{
			break;
		}
		else
		{
			latitude = atof(token);
		}
			token = strtok(NULL, ",");
	}

		return latitude;
}

double get_lon(char *gga)
{
	double longitude = 0.0;

	char gga_cpy[256];
	strncpy(gga_cpy, gga, 256);

	char *token = strtok(gga_cpy, ",");

	while (token != NULL)
	{
		if ((strcmp(token, "W") == 0) || (strcmp(token, "E") == 0))
		{
			break;
		}
		else
		{
			longitude = atof(token);
		}
			token = strtok(NULL, ",");
	}

		return longitude;
}

void get_time(char *gga, char *time)
{
	int hours, minutes, seconds;
	char *comma1 = strchr(gga, ',');
	char *comma2 = strchr(comma1 + 1, ',');

	size_t length = comma2 - (comma1 + 1);

	strncpy(time, comma1 + 1, length);
	time[length] = '\0';

	sscanf(time, "%2d%2d%2d", &hours, &minutes, &seconds);

	hours -= 5;

	if (hours < 0) {
		hours += 24;
	}

	sprintf(time, "%02d:%02d:%02d\n", hours, minutes, seconds);
}

double get_alt(char *gga)
{
	double altitude = 0.0;

	char gga_cpy[256];
	strncpy(gga_cpy, gga, 256);

	char *token = strtok(gga_cpy, ",");

	while (token != NULL)
	{
		if (strcmp(token, "M") == 0)
		{
			break;
		}
		else
		{
			altitude = atof(token);
		}
			token = strtok(NULL, ",");
	}

		return altitude;
}

double ddm2dd(double ddm) {
    double degrees = floor(ddm / 100.0);
    double minutes = ddm - degrees * 100.0;
    double dd = degrees + minutes / 60.0;
    return dd;
}

// Function to convert degrees to radians
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Function to calculate the distance between two coordinates using Haversine formula
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
