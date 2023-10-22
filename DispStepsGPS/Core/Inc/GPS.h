/*
 * GPS.h
 *
 *  Created on: Oct 22, 2023
 *      Author: martinguarnieri
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_


double ddm2dd(double ddm);
double degreesToRadians(double degrees);
double calculateDistance(double lat1, double lon1, double lat2, double lon2);


//double get_lat(char *gga);
//double get_lon(char *gga);
//double get_alt(char *gga);

#endif /* INC_GPS_H_ */
