## hello world application 

from flask import Flask 

app = Flask(__name__)


@app.route('/')
def hello():
    return 'Hello World test 1!'