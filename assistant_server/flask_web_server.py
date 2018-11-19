from flask import Flask, render_template, request, url_for
from auth import auth_error


app = Flask(__name__)
app.jinja_env.auto_reload = True
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.secret_key = "super secret key"


@app.route('/', methods=["GET", "POST"])
def start():
    cookie_username = request.cookies.get('username')
    return render_template("main.html", mssg = cookie_username)


@app.route('/login', methods=["GET", "POST"])
def login():
    if request.method == 'POST':
        if not auth_error(request.form.get("user_name"), request.form.get("password")):
            # Authentication successful.
            return render_template("search.html")
        else:
            return render_template('main.html', mssg = "Wrong passwod or userame, try again.")
        # mssg = "Wrong password or username, try again."
    return render_template('main.html')


if __name__ == '__main__':
    app.run(debug = True)
