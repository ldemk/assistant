from db_assistant import Database
import datetime
from hash_function import hash_password


class AuthenticationDB(Database):
    def __init__(self, user_email, user_key):
        Database.__init__(self, user_email, user_key)
        self.authentication = '/authentication'
        self.users = '/users'
        self.passwords_id = '/passwords_id'
        self.valid_dates_id = '/valid_dates_id'

    def _user_password_id(self, user):
        return self.database.get('{}/{}/{}'.format(
            self.authentication, self.users, user), None)['password_id']

    def _user_password(self, user):
        password_id = self._user_password_id(user)
        password = self.database.get('{}/{}/{}'.format(
            self.authentication, self.passwords_id, password_id), None)['password']
        return password

    def _user_valid_date_id(self, user):
        return self.database.get('{}/{}/{}'.format(
            self.authentication, self.users, user), None)['valid_date_id']

    def _user_valid_date(self, user):
        valid_date_id = self._user_valid_date_id(user)
        valid_date = self.database.get('{}/{}/{}'.format(
            self.authentication, self.valid_dates_id, valid_date_id), None)
        return valid_date

    @staticmethod
    def compare_dates(date1, date2):
        """
        :param date?: in format DD/MM/YYYY
        :return: -1 if date1 < date2, 0 if date1 == date2, 1 if date1 > date2
        """
        date1_int = ''.join(date1.split('/')[-1::-1])
        date2_int = ''.join(date2.split('/')[-1::-1])
        if date1_int < date2_int:
            return -1
        elif date1_int > date2_int:
            return 1
        else:
            return 0

    def _check_valid_date(self, user):
        valid_date = self._user_valid_date(user)
        now = datetime.datetime.now()
        now_date = '{}/{}/{}'.format(str(now.day).zfill(2), str(now.month).zfill(2), now.year)
        if self.compare_dates(now_date, valid_date['from']) == -1:
            raise TooEarlyRequest
        if self.compare_dates(now_date, valid_date['to']) == 1:
            raise TermOfUseExpired

    def check_user_password(self, user, password):
        """
        :param user: username
        :param password: entered password
        :return: check if entered password corresponds to registered.
                 Some errors may occur.
        """
        if self.database.get('{}/{}/{}'.format(self.authentication, self.users, user), None) is None:
            raise UserDoesNotExists
        self._check_valid_date(user)
        if self._user_password(user) == hash_password(password):
            return True
        raise PasswordDoesNotMatch

    def delete_user(self, user):
        """
        :param user: username waiting to be deleted
        :return: None
        """
        user_id = self.database.get('{}/{}/{}'.format(
            self.authentication, self.users, user), None)
        password_id = user_id['password_id']
        valid_date_id = user_id['valid_date_id']
        self.database.delete('{}/{}'.format(self.authentication, self.users), user)
        self.database.delete('{}/{}'.format(self.authentication, self.passwords_id), password_id)
        self.database.delete('{}/{}'.format(self.authentication, self.valid_dates_id), valid_date_id)


class UserDoesNotExists(Exception):
    pass


class PasswordDoesNotMatch(Exception):
    pass


class TooEarlyRequest(Exception):
    pass


class TermOfUseExpired(Exception):
    pass
