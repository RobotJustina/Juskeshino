import yaml
import rospkg
import rospy
import random
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Quaternion
from tf.transformations import quaternion_from_euler

import tf2_ros
x_ofs = 0.3
y_ofs = 4.6

class RECEPTIONIST:
    def __init__(self, knowledge_file='/receptionist_knowledge.yaml'):
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path('config_files') + knowledge_file
        self.informacion_fiesta = self.load_data_from_yaml()
        rospy.Subscriber('/analyze_result', String, self.callback)
        #self.last_seat_assigned = 'None'  # Place_0, Place_1, Place_2
        self.active_guest = 'None'  # Guest_0, Guest_1, Guest_2
        self.active_seat = 'None' # Place_0, Place_1, Place_2

        # TF2_ROS publisher
        self.tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.StaticTransformBroadcaster()

    # --- YAML read and write ---
    def load_data_from_yaml(self):
        with open(self.file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data

    def save_data_to_yaml(self, new_knowledge):
        with open(self.file_path, 'w') as file:
            documents = yaml.dump(new_knowledge, file)

    # --- Add ---
    # Adds a new person to party context, updates active guest and pre-assign a seat to this new guest
    def add_guest(self, name, drink = 'None'):
        self.active_guest = self.add_new_person(name, drink)
        self.active_seat = self.get_any_available_seat()

    # Adds description to active guest
    def add_guest_description(self, description):
        if self.active_guest != 'None':
            self.informacion_fiesta['People'][self.active_guest]['description'] = description
            self.save_data_to_yaml(self.informacion_fiesta)
    
    # Adds a drink to active guest
    def add_guest_drink(self, drink):
        self.informacion_fiesta['People'][self.active_guest]['drink'] = drink
    
    # Adds a new person to party context, in case a intruder is on party scene
    def add_new_person(self, name, drink):
        guests_len = len(self.informacion_fiesta['People'])
        guest_num = f'Guest_{guests_len}'
        self.informacion_fiesta['People'][guest_num] = {
            'drink': drink,
            'location': 'None',
            'name': name,
            'description': 'None'}
        self.save_data_to_yaml(self.informacion_fiesta)
        return guest_num

    # --- Gets ---
    # Gets randomly an available seat from available seats list
    def get_any_available_seat(self):
        seats = self.get_available_seats()
        if len(seats) > 0:
            return random.choice(seats)
        else:
            return 'None'

    # Gets the available seats list
    def get_available_seats(self):
        available_seats = []
        for place, info in self.informacion_fiesta['Places'].items():
            if info['occupied'] == "None":
                available_seats.append(place)
        return available_seats

    # Gets guest number with guest name
    def get_guest_by_name(self, guest_name):
        for guest, info in self.informacion_fiesta['People'].items():
            if info['name'] == guest_name:
                return guest
        return 'None'

    # Gets the seat number with guest name (unused)
    def get_guest_seat(self, guest_name):
        for place, info in self.informacion_fiesta['Places'].items():
            if info['occupied'] == guest_name:
                return place
        return 'None'
    
    def get_active_guest_drink(self):
        return self.informacion_fiesta['People'][self.active_guest]['drink']


    # Gets every place X, Y, Theta and place number on party context
    def get_places_location(self):
        locs = []
        places = []
        for place, info in self.informacion_fiesta['Places'].items():
            if place != 'Place_0':
                xyt = [info['location']['x'] + x_ofs, info['location']['y'] + y_ofs, info['location']['theta']]
                places.append(place)
                locs.append(xyt)
        return places, locs
    
    def get_active_seat(self):
        if self.active_seat != 'None':
            return True, self.active_seat
        else:
            return False, self.active_seat

    # Gets X, Y, Theta location of active seat, if there is no active seat returns False
    def get_active_seat_location(self):
        if self.active_seat != 'None':
            return True, [self.informacion_fiesta['Places'][self.active_seat]['location']['x'],
                    self.informacion_fiesta['Places'][self.active_seat]['location']['y'],
                    self.informacion_fiesta['Places'][self.active_seat]['location']['theta']]
        else:
            return False, [0,0,0]
    
    # Gets every place number on party context (unused)
    def get_places(self):
        return list(self.informacion_fiesta['Places'].keys())
    
    def get_guests_seat_assignments(self):
        seats = {}
        for place, info in self.informacion_fiesta['Places'].items():
            if info['occupied'] != 'None':
                if place != 'Place_0':
                    seats[place] = info['occupied']
        return seats

    #Gets active guest description
    def get_active_guest_description(self):
        if self.active_guest != 'None':
            return self.informacion_fiesta['People'][self.active_guest]['description']
        else:
            return 'No active guest found'

    # Gets host name and location
    def get_host_info(self):
        host = self.informacion_fiesta['People']['Guest_0']
        print(host)
        return host['name'], host['location']
    
    #Gets active guest
    def get_active_guest(self):
        return self.active_guest
    
    #Gets active guest name
    def get_active_guest_name(self):
        if self.active_guest != 'None':
            return self.informacion_fiesta['People'][self.active_guest]['name']
        else:
            return 'None'

    # --- Seat methods ---
    def seat_confirmation(self, guest_found = 'None'):
        #This means that there is no one on this seat and it is free to seat active guest here
        if guest_found == 'None':
            self.assign_seat_to_guest(self.active_guest, self.active_seat)
        #This means that there is someone on this seat and a new assign is needed
        else:
            #Verify if the guest found is part of our party
                #If the guest is part of our party update their seat
                #if not, add them to out party and assign them this seat
                #finally get again an available seat to active guest
            guest_num = self.get_guest_by_name(guest_found)
            isGuest = guest_num != 'None'
            if isGuest:
                self.update_seat_assignment(guest_num)
            else:
                guest_num = self.add_new_person(guest_found, drink='None')
                self.assign_seat_to_guest(guest_num, self.active_seat)
                self.active_seat = self.get_any_available_seat()


    def assign_seat_to_guest(self, guest, seat):
        #Update data
        self.informacion_fiesta['Places'][seat]['occupied'] = guest
        self.informacion_fiesta['People'][guest]['location'] = seat
        self.save_data_to_yaml(self.informacion_fiesta)

    def update_seat_assignment(self, guest_num):
        #get last known seat for guest found
        last_known_seat = self.informacion_fiesta['People'][guest_num]['location']

        #update data
        if last_known_seat != 'None':
            self.informacion_fiesta['Places'][last_known_seat]['occupied'] = 'None'
        self.informacion_fiesta['Places'][self.active_seat]['occupied'] = guest_num
        self.informacion_fiesta['People'][guest_num]['location'] = self.active_seat
        self.save_data_to_yaml(self.informacion_fiesta)

    def clean_knowledge(self, host_name, host_location):
        host = self.informacion_fiesta['People']['Guest_0']
        host['name'] = host_name
        host['location'] = host_location
        self.informacion_fiesta['People'] = {'Guest_0': host}
        for place, info in self.informacion_fiesta['Places'].items():
            if place == host_location:
                info['occupied'] = host_name
            elif place == 'Place_0':
                info['occupied'] = 'Not_available'
            else:
                info['occupied'] = 'None'
        self.save_data_to_yaml(self.informacion_fiesta)

    def callback(self, msg):
        takeshi_line = msg.data
        if len(takeshi_line) > 2:
            self.add_guest_description(takeshi_line)

    def publish_tf_seats(self):
        seat_transform = TransformStamped()
        face_seat_transform = TransformStamped()
        
        seat_transform.header.stamp = rospy.Time(0)
        seat_transform.header.frame_id = "map"

        face_seat_transform.header.stamp = rospy.Time(0)

        places, locs = self.get_places_location()

        for place, loc in zip(places, locs):
            # Publish seats tf
            seat_transform.child_frame_id = place
            seat_transform.transform.translation.x = loc[0]
            seat_transform.transform.translation.y = loc[1]
            #seat_transform.transform.translation.z = 0.0
            seat_transform.transform.rotation = Quaternion(*quaternion_from_euler(0,0,loc[2]))
            self.br.sendTransform(seat_transform)

            # Publish an approximation of face position of a seat
            face_seat_transform.header.frame_id = place
            face_seat_transform.child_frame_id = place.replace('_', '_face')
            face_seat_transform.transform.translation.x = 0.5
            #face_seat_transform.transform.translation.z = 1.0
            face_seat_transform.transform.rotation.w = 1
            self.br.sendTransform(face_seat_transform)
        return True