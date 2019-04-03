def hash_password(password):
    hashed = list(password)[-1::-1]
    hashed.append(str(sum([ord(c) for c in password])))
    return ''.join(hashed)
