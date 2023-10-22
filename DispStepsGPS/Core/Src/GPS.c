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
