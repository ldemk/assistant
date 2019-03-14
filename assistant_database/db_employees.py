import admin_db_employees


class EmployeesDB:
    def __init__(self, user_email, user_key):
        self.__admin = admin_db_employees.AdminDBEmployees(user_email, user_key)

    def employee_is_available(self, name, surname):
        return self.__admin.employee_is_available(name, surname)

    def location_info(self, name, surname):
        return self.__admin.location_info(name, surname)

    def coordinate_x(self, name, surname):
        return self.__admin.coordinate_x(name, surname)

    def coordinate_y(self, name, surname):
        return self.__admin.coordinate_y(name, surname)
