'''Python script to create a kml file from a .TXT file that has (latitude, longitude, time) coordinates.
the kml file can then be looked at on Google Earth/Maps'''
import simplekml
import sys

def read_coordinates(file_path):
    coordinates_and_time = []
    with open(file_path, 'r') as file:
        for line in file:
            lat_str, lon_str, time_str = line.strip().split(',')
            lat = convert_coordinate(lat_str)
            lon = convert_coordinate(lon_str)
            #round coordinates to 6 decimal places and append to list
            coordinates_and_time.append((round(lon, 6), round(lat, 6), time_str))
    return coordinates_and_time

def convert_coordinate(coord_str):
    last_char =  coord_str[-1] #check if N or E
    direction = 1 if last_char == 'N' or last_char == 'E' else -1
    coord_value = float(coord_str[:-1])
    # Convert from degree minute format to decimal degree format
    return round(abs(coord_value // 100 + coord_value % 100 / 60) * direction, 6)


def create_kml(coordinates_and_time):
    output_file = sys.argv[2] #second command line argument
    kml = simplekml.Kml() #create kml file

    # KML linestring from coordinates list
    path = kml.newlinestring(name='Coordinates Path')
    path.coords = [(lon, lat) for lon, lat, _ in coordinates_and_time]
    path.style.linestyle.color = simplekml.Color.red  # Make line red
    path.style.linestyle.width = 4  # Make line thicker

    # KML points to put on line
    for i, (lon, lat, time) in enumerate(coordinates_and_time, start=1):
        point = kml.newpoint(name=f'Point {i}', coords=[(lon, lat)])
        point.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png'  # URL for small circle
        point.style.iconstyle.scale = 0.5  # Set dot size
        point.description = f'Latitude: {lat}, Longitude: {lon}, Time: {time}'

    # save kml file
    kml.save(output_file)
    print(f"KML file saved to {output_file}")


if __name__ == "__main__":
    #error check for correct command line arguenments
    if len(sys.argv) < 3:
        print("input .TXT file and output .kml file name required")
        exit()
    elif sys.argv[1][-4:] != ".TXT":
        print("Input file needs to have .TXT extension")
        exit()
    elif sys.argv[2][-4:] != ".kml":
        print("Ouptut file needs to have .kml")
        exit()
    file_path = sys.argv[1]#first command line argument 
    coordinates_with_time = read_coordinates(file_path)
    create_kml(coordinates_with_time)

