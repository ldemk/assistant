from admin_db_authentication import AdminDBAuthentication


class AuthenticationDB:
    def __init__(self, user_email, user_key):
        self.__admin = AdminDBAuthentication(user_email, user_key)

    def check_user_password(self, username, password):
        """
        :param username: username
        :param password: entered password
        :return: check if entered password corresponds to registered.
                 Some errors may occur.
        """
        self.__admin.check_user_password(username, password)

    def delete_user(self, username):
        """
        :param username: username waiting to be deleted
        :return: None
        """
        self.__admin.delete_user(username)
