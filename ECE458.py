from flask import Flask, render_template, request, jsonify
import flask_excel as excel
import pandas as pd

app = Flask(__name__, template_folder='templates')
excel.init_excel(app)


@app.route("/")
def home():
		return "Hello, World!"

@app.route("/upload", methods=['GET', 'POST'])
def upload_file():
    if request.method == 'POST':
        print(request.files['file'])
        f = request.files['file'] #Can you replace with actual file? 
        
        data_xls = pd.read_excel(f)
        return data_xls.to_html()
    return '''
    <!doctype html>
    <title>Upload an excel file</title>
    <h1>Excel file upload (csv, tsv, csvz, tsvz only)</h1>
    <form action="" method=post enctype=multipart/form-data>
    <p><input type=file name=file><input type=submit value=Upload>
    </form>
    '''

@app.route("/export", methods=['GET'])
def export_records():
    return 

if __name__ == "__main__":
    app.run()
@app.route("/track")
def trackerApp():
		return "TRACKING" 
		
@app.route("/show", methods= ["GET", "POST"])
def index():
	return render_template('index.html')
    
@app.route("/show#home")
def nope():
    return "HOMEPAGE"
    
	
if __name__ == "__main__":
	app.run(host='10.0.0.172', port=8080,use_reloader=False, debug=True) #Change host, local IPV4 of Melanie's Machine 
