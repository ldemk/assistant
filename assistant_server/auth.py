def auth_error(username, password):
    """
    Function to perform user authentication and return an error code.
    """
    #  imitation of future request to the firebase.
    db = {"first": "Veo", "second": "Bix", "third": "QaAJ"}
    try:
        if db[username] == password:
            return 0
        return -2
    except KeyError:
        return -1
