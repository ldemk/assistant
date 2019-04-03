from assistant_database.db_authentication import AuthenticationDB
from assistant_database.db_authentication import UserDoesNotExists
from assistant_database.db_authentication import PasswordDoesNotMatch
from assistant_database.db_authentication import TooEarlyRequest
from assistant_database.db_authentication import TermOfUseExpired

from assistant_database import config


def auth_check(username, password):
    """
    Function to perform user authentication and return an error code.
    """
    db = AuthenticationDB(config.my_email, config.my_key)
    try:
        if db.check_user_password(username, password):
            print("yess")
            return 0
    except UserDoesNotExists:
        return -1
    except PasswordDoesNotMatch:
        return -2
    except TooEarlyRequest:
        return -3
    except TermOfUseExpired:
        db.delete_user(username)
        return -4
