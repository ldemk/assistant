def auth_error(username, password):
    """
    Function to perform user authentication and return an error code.
    """
    db = {"quaternions": "Veo", "second": "Bix", "third": "QaAJ"}     # imitation of future request to the firebase.
    try:
        if db[username] == password:
            print("yess")
            return 0
        return -2
    except KeyError:
        return -1


def get_coords(name):
    if name == "Brad Thaxton":
        coords = {'x': -2.16564114295, 'y': -5.48124138424}
    else:
        coords = {'x': -5.16564114295, 'y': -1.48124138424}

        return coords