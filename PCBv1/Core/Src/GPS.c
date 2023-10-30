/*
 * GPS.c
 *
 *  Created on: Oct 22, 2023
 *      Author: martinguarnieri
 */

//#include "GPS.h"
//#include <string.h>
//#include "math.h"
//
//double get_lat(char *gga)
//{
//	double latitude = 0.0;
//
//	char gga_cpy[256];
//	strncpy(gga_cpy, gga, 256);
//
//	char *token = strtok(gga_cpy, ",");
//
//	while (token != NULL)
//	{
//		if ((strcmp(token, "N") == 0) || (strcmp(token, "S") == 0))
//		{
//			break;
//		}
//		else
//		{
//			latitude = atof(token);
//		}
//			token = strtok(NULL, ",");
//	}
//
//		return latitude;
//}
//
//double get_lon(char *gga)
//{
//	double longitude = 0.0;
//
//	char gga_cpy[256];
//	strncpy(gga_cpy, gga, 256);
//
//	char *token = strtok(gga_cpy, ",");
//
//	while (token != NULL)
//	{
//		if ((strcmp(token, "W") == 0) || (strcmp(token, "E") == 0))
//		{
//			break;
//		}
//		else
//		{
//			longitude = atof(token);
//		}
//			token = strtok(NULL, ",");
//	}
//
//		return longitude;
//}
//
//double get_alt(char *gga)
//{
//	double altitude = 0.0;
//
//	char gga_cpy[256];
//	strncpy(gga_cpy, gga, 256);
//
//	char *token = strtok(gga_cpy, ",");
//
//	while (token != NULL)
//	{
//		if (strcmp(token, "M") == 0)
//		{
//			break;
//		}
//		else
//		{
//			altitude = atof(token);
//		}
//			token = strtok(NULL, ",");
//	}
//
//		return altitude;
//}

#include <stdio.h>
#include <math.h>
#define RADIUS_OF_EARTH 6371000 // Earth's radius in meters

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
