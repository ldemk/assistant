def auth_error(username, password):
    """
    Function to perform user authentication and return an error code.
    """
    db = {"first": "Veo", "second": "Bix", "third": "QaAJ"}     # imitation of future request to the firebase.
    try:
        if db[username] == password:
            print("yess")
            return 0
        return -2
    except KeyError:
        return -1