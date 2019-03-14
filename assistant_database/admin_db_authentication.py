from db_assistant import Database
import datetime
from hash_function import hash_password


class AdminDBAuthentication(Database):
    def __init__(self, my_email, my_key):
        Database.__init__(self, my_email, my_key)
        self.authentication = '/authentication'
        self.users = 'users'
        self.passwords_id = 'passwords_id'
        self.valid_dates_id = 'valid_dates_id'

    def _user_info(self, username):
        return self.database.get('{}/{}/{}'.format(self.authentication, self.users, username), None)

    def _user_password_id(self, username):
        return self._user_info(username)['password_id']

    def _user_valid_date_id(self, username):
        return self._user_info(username)['valid_date_id']

    def _user_privilege(self, username):
        return self._user_info(username)['privilege']

    def _user_password(self, username):
        password_id = self._user_password_id(username)
        password = self.database.get('{}/{}/{}'.format(
            self.authentication, self.passwords_id, password_id), None)['password']
        return password

    def _user_valid_date(self, username):
        valid_date_id = self._user_valid_date_id(username)
        valid_date = self.database.get('{}/{}/{}'.format(
            self.authentication, self.valid_dates_id, valid_date_id), None)
        return valid_date

    def _delete_username(self, username):
        self.database.delete('{}/{}'.format(self.authentication, self.users), username)

    def _delete_password_id(self, password_id):
        self.database.delete('{}/{}'.format(self.authentication, self.passwords_id), password_id)

    def _delete_valid_date_id(self, valid_date_id):
        self.database.delete('{}/{}'.format(self.authentication, self.valid_dates_id), valid_date_id)

    def delete_user(self, username):
        """
        :param username: username waiting to be deleted
        :return: None
        """
        user_info = self._user_info(username)
        password_id = user_info['password_id']
        valid_date_id = user_info['valid_date_id']
        self._delete_password_id(password_id)
        self._delete_valid_date_id(valid_date_id)
        self._delete_username(username)

    def update_password(self, username, new_password):
        self.database.put('{}/{}'.format(self.authentication, self.passwords_id),
                          self._user_password_id(username), {u'password': hash_password(new_password)})

    def update_valid_date(self, username, new_valid_dates):
        self.database.put('{}/{}'.format(self.authentication, self.valid_dates_id),
                          self._user_valid_date_id(username), {u'from': new_valid_dates['from'],
                                                               u'to': new_valid_dates['to']})

    def update_privilege(self, username, privilege):
        self.database.put('{}/{}/{}'.format(self.authentication, self.users, username),
                          u'privilege',  privilege)

    def _next_id(self):
        next_id = int(self.database.get('{}'.format(self.authentication), None)['current_user_id'])
        next_id = str(next_id + 1).zfill(3)
        self.database.put('{}'.format(self.authentication), u'current_user_id',  next_id)
        return next_id

    def add_user(self, user):
        """
        Adding user to database
        :param user: dict with params
                    {'username': obligatory,
                    'password': obligatory,
                    'privilege': obligatory ('guest' / 'admin'),
                    'valid_date': {'from': default -- today
                                    'to': default -- tomorrow}}
        :return: True if user added, False -- otherwise
        """
        if not ('username' in user and 'password' in user and 'privilege' in user):
            return False
        current_id = self._next_id()
        self.database.put('{}/{}'.format(self.authentication, self.users), user['username'],
                          {u'password_id': current_id,
                           u'valid_date_id': current_id,
                           u'privilege': user['privilege']})
        self.database.put('{}/{}'.format(self.authentication, self.passwords_id), current_id,
                          {u'password': hash_password(user['password'])})
        if 'valid_date' not in user:
            now = datetime.datetime.now()
            today = '{}/{}/{}'.format(str(now.day).zfill(2), str(now.month).zfill(2), now.year)
            tomorrow = '{}/{}/{}'.format(str(now.day + 1).zfill(2), str(now.month).zfill(2), now.year)
            user['valid_date'] = {'from': today, 'to': tomorrow}
        self.database.put('{}/{}'.format(self.authentication, self.valid_dates_id), current_id,
                          {u'from': user['valid_date']['from'],
                           u'to': user['valid_date']['to']})
        return True

    def _check_privilege(self, username, privilege):
        user_privilege = self._user_info(username)['privilege']
        if not user_privilege == privilege:
            raise NotRequiredPrivilege

    def log_in(self, username, password):
        """
        :param username: username
        :param password: password
        :return: None / Error
        """
        self.check_user_password(username, password)
        self._check_privilege(username, 'admin')

    def _check_valid_date(self, username):
        valid_date = self._user_valid_date(username)
        now = datetime.datetime.now()
        now_date = '{}/{}/{}'.format(str(now.day).zfill(2), str(now.month).zfill(2), now.year)
        if self.compare_dates(now_date, valid_date['from']) == -1:
            raise TooEarlyRequest
        if self.compare_dates(now_date, valid_date['to']) == 1:
            raise TermOfUseExpired

    def check_user_password(self, username, password):
        """
        :param username: username
        :param password: entered password
        :return: check if entered password corresponds to registered.
                 Some errors may occur.
        """
        if self._user_info(username) is None:
            raise UserDoesNotExists
        self._check_valid_date(username)
        if self._user_password(username) == hash_password(password):
            return True
        raise PasswordDoesNotMatch

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


class UserDoesNotExists(Exception):
    pass


class PasswordDoesNotMatch(Exception):
    pass


class TooEarlyRequest(Exception):
    pass


class TermOfUseExpired(Exception):
    pass


class NotRequiredPrivilege(Exception):
    pass
