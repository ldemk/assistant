from db_assistant import Database


class EmployeesDB(Database):
    def __init__(self, user_email, user_key):
        Database.__init__(self, user_email, user_key)
        self.employees = '/employees'
        self.locations = '/locations_id'
        self.coordinates = '/coordinates_id'

    def _employee_location_id(self, first_name, last_name):
        employee_id = self.database.get('{}/{} {}'.format(self.employees, first_name, last_name), None)
        return employee_id['location_id']

    def employee_is_available(self, first_name, last_name):
        """
        :param first_name: (str)
        :param last_name: (str)
        :return: (bool) if employee is available
        """
        employee_id = self.database.get('{}/{} {}'.format(self.employees, first_name, last_name), None)
        return employee_id['is available']

    def location_info(self, first_name, last_name):
        """
        :param first_name: (str)
        :param last_name: (str)
        :return: (dict) {u'office': UNKNOWN, u'floor': UNKNOWN, u'workplace': UNKNOWN,})
        """
        location_id = self._employee_location_id(first_name, last_name)
        location = self.database.get('{}/{}'.format(self.locations, location_id), None)
        return location['info']

    def _coordinate_id(self, location_id):
        coordinate_id = self.database.get('{}/{}/{}'.format(self.locations, location_id, 'coordinates_id'), None)
        return coordinate_id

    def _coordinates(self, first_name, last_name):
        location_id = self._employee_location_id(first_name, last_name)
        coordinate_id = self._coordinate_id(location_id)
        coordinates = self.database.get('{}/{}'.format(self.coordinates, coordinate_id), None)
        return coordinates

    def coordinate_x(self, first_name, last_name):
        """
        :param first_name: (str)
        :param last_name: (str)
        :return: coordinate x of employee
        """
        return self._coordinates(first_name, last_name)['x']

    def coordinate_y(self, first_name, last_name):
        """
        :param first_name: (str)
        :param last_name: (str)
        :return: coordinate y of employee
        """
        return self._coordinates(first_name, last_name)['y']
