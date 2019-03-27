import firebase


class Database:
    def __init__(self, user_email, user_key):
        self.database = firebase.FirebaseApplication('https://assistant-c2e8c.firebaseio.com/', authentication=None)
        authentication = firebase.FirebaseAuthentication(user_key, user_email)
        firebase.authentication = authentication
