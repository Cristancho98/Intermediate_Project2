from flask import Flask, render_template, request, url_for, redirect
from flask_sqlalchemy import SQLAlchemy
import datetime

app = Flask(__name__)

app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///database/Temp.db'

db = SQLAlchemy(app)

class Temp(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    temperature = db.Column(db.String(20))
    date = db.Column(db.String(20))

@app.route('/')
def home():
    while True:
        temps = Temp.query.all()
        actual = int(input("la temperatura es:"))
        return render_template('index.html', temps = temps, actual=actual)

@app.route('/create-temp', methods=['POST'])
def create():
    new_temp = Temp(content=request.form['content'], date= datetime.datetime.now())
    db.session.add(new_temp)
    db.session.commit()
    return redirect(url_for('home'))

@app.route('/done/<id>')
def done(id):
    temp = Temp.query.filter_by(id=int(id)).first()
    temp.done = not(temp.done)
    db.session.commit()
    return redirect(url_for('home'))

@app.route('/deleteall')
def delete():
    db.execute("DELETE FROM Temp")
    db.session.commit()
    return redirect(url_for('home'))

@app.route('/turnoff',methods=['POST'])
def turnoff():
    Temp.query.filter_by(id=int(id)).delete()
    db.session.commit()
    return redirect(url_for('home'))

    

if __name__ == '__main__':
    app.run(debug=True)
