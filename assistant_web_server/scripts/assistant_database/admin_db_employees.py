from db_assistant import Database


class AdminDBEmployees(Database):
    def __init__(self, user_email, user_key):
        Database.__init__(self, user_email, user_key)
        self.employees = '/employees'
        self.locations = '/locations_id'
        self.coordinates = '/coordinates_id'

    def _if_user_exists(self, name, surname):
        return self.database.get('{}/{}/{}'.format(self.employees, self.employees,
                                                   self._whole_name(name, surname)), None) is not None

    @staticmethod
    def _whole_name(name, surname):
        return '{}_{}'.format(name, surname).lower()

    def _next_id(self):
        next_id = int(self.database.get('{}'.format(self.employees), None)['current_user_id'])
        next_id = str(next_id + 1).zfill(3)
        self.database.put('{}'.format(self.employees), u'current_user_id', next_id)
        return next_id

    def add_employee(self, employee):
        """
        Adding employee to database.
        :param employee: dict with params:
                        {'name': obligatory,
                         'surname': obligatory,
                         'is_available': default -- True,
                         'office': default -- 'Lviv HQ',
                         'floor': obligatory,
                         'workplace': obligatory,
                         'x': obligatory,
                         'y': obligatory}
        :return: True if employee added, False -- otherwise
        """
        if not ('name' in employee and 'surname' in employee and 'floor' in employee and 'workplace' in employee and 'x'
                in employee and 'y' in employee):
            return False
        if self._if_user_exists(employee['name'], employee['surname']):
            raise EmployeeExists
        current_id = self._next_id()
        if 'office' not in employee:
            employee['office'] = 'Lviv HQ'
        if 'is_available' not in employee:
            employee['is_available'] = True
        self.database.put('{}/{}'.format(self.employees, self.employees),
                          self._whole_name(employee['name'], employee['surname']),
                          {u'is_available': employee['is_available'],
                           u'location_id': current_id})
        self.database.put('{}/{}'.format(self.employees, self.locations), current_id,
                          {u'coordinate_id': current_id,
                           u'info': {u'office': employee['office'],
                                     u'floor': employee['floor'],
                                     u'workplace': employee['workplace']}})
        self.database.put('{}/{}'.format(self.employees, self.coordinates), current_id,
                          {u'x': employee['x'],
                           u'y': employee['y']})

    def _employee_location_id(self, name, surname):
        employee_id = self.database.get('{}/{}/{}'.format(self.employees, self.employees,
                                                          self._whole_name(name, surname)), None)
        return employee_id['location_id']

    def employee_is_available(self, name, surname):
        """
        :param name: (str)
        :param surname: (str)
        :return: (bool) if employee is available
        """
        if not self._if_user_exists(name, surname):
            raise EmployeeDoesNotExists
        employee_id = self.database.get('{}/{}/{}'.format(self.employees, self.employees,
                                                          self._whole_name(name, surname)), None)
        return employee_id['is_available']

    def location_info(self, name, surname):
        """
        :param name: (str)
        :param surname: (str)
        :return: (dict) {u'office': UNKNOWN, u'floor': UNKNOWN, u'workplace': UNKNOWN,})
        """
        location_id = self._employee_location_id(name, surname)
        location = self.database.get('{}/{}/{}'.format(self.employees, self.locations, location_id), None)
        return location['info']

    def _coordinate_id(self, location_id):
        coordinate_id = self.database.get('{}/{}/{}'.format(self.employees, self.locations, location_id), None)
        return coordinate_id['coordinate_id']

    def _coordinates(self, name, surname):
        location_id = self._employee_location_id(name, surname)
        coordinate_id = self._coordinate_id(location_id)
        coordinates = self.database.get('{}/{}/{}'.format(self.employees, self.coordinates, coordinate_id), None)
        return coordinates

    def coordinate_x(self, name, surname):
        """
        :param name: (str)
        :param surname: (str)
        :return: coordinate x of employee
        """
        return self._coordinates(name, surname)['x']

    def coordinate_y(self, name, surname):
        """
        :param name: (str)
        :param surname: (str)
        :return: coordinate y of employee
        """
        return self._coordinates(name, surname)['y']

    def _delete_name_surname(self, name, surname):
        self.database.delete('{}/{}'.format(self.employees, self.employees), self._whole_name(name, surname))

    def _delete_employee_location_id(self, location_id):
        self.database.delete('{}/{}'.format(self.employees, self.locations), location_id)

    def _delete_employee_coordinate_id(self, coordinate_id):
        self.database.delete('{}/{}'.format(self.employees, self.coordinates), coordinate_id)

    def delete_employee(self, name, surname):
        if not self._if_user_exists(name, surname):
            raise EmployeeDoesNotExists
        location_id = self._employee_location_id(name, surname)
        coordinate_id = self._coordinate_id(location_id)
        self._delete_employee_coordinate_id(coordinate_id)
        self._delete_employee_location_id(location_id)
        self._delete_name_surname(name, surname)

    def update_is_available(self, name, surname, is_available):
        if not self._if_user_exists(name, surname):
            raise EmployeeDoesNotExists
        self.database.put('{}/{}/{}'.format(self.employees, self.employees,
                                            self._whole_name(name, surname)), u'is_available',  is_available)

    def update_office(self, name, surname, new_office):
        if not self._if_user_exists(name, surname):
            raise EmployeeDoesNotExists
        self.database.put('{}/{}/{}'.format(self.employees, self.locations,
                          self._employee_location_id(name, surname)), u'info/office', new_office)

    def update_floor(self, name, surname, new_floor):
        if not self._if_user_exists(name, surname):
            raise EmployeeDoesNotExists
        self.database.put('{}/{}/{}'.format(self.employees, self.locations,
                          self._employee_location_id(name, surname)), u'info/floor', new_floor)

    def update_workplace(self, name, surname, new_workplace):
        if not self._if_user_exists(name, surname):
            raise EmployeeDoesNotExists
        self.database.put('{}/{}/{}'.format(self.employees, self.locations,
                          self._employee_location_id(name, surname)), u'info/workplace', new_workplace)

    def update_x_coordinate(self, name, surname, new_x):
        if not self._if_user_exists(name, surname):
            raise EmployeeDoesNotExists
        self.database.put('{}/{}/{}'.format(self.employees, self.coordinates,
                          self._coordinate_id(self._employee_location_id(name, surname))), u'x', new_x)

    def update_y_coordinate(self, name, surname, new_y):
        if not self._if_user_exists(name, surname):
            raise EmployeeDoesNotExists
        self.database.put('{}/{}/{}'.format(self.employees, self.coordinates,
                          self._coordinate_id(self._employee_location_id(name, surname))), u'y', new_y)


class EmployeeExists(Exception):
    pass


class EmployeeDoesNotExists(Exception):
    pass
