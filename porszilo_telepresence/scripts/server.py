#! /usr/bin/env python

from flask import Flask, render_template, flash, session, redirect, url_for
app = Flask(__name__, template_folder="../templates")

@app.route("/")
def index():
    return render_template("index.html")

host = "localhost"
port = "5000"

if __name__ == "__main__":
    app.run(host=host, port=port, debug=True)
