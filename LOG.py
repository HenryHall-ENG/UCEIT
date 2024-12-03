from datetime import datetime
import os
import sqlite3
import json

class logger():
    def __init__(self):
        self.isRunning = False
        self.path = r'C:\Users\henri\OneDrive\Documents\EIT\logging'
    
    def storeData(self, data):
        if self.isRunning == True:
            self.timestamp = datetime.now().strftime('%d-%m-%Y %H:%M:%S')
            self.cursor.execute("INSERT INTO measurements (timestamp, data) VALUES (?, ?)", (self.timestamp, json.dumps(data)))
            self.conn.commit()

    def getData(self):
        self.cursor.execute("SELECT * FROM measurements")
        rows = self.cursor.fetchall()
        return rows
    
    def startLog(self):
        self.timestamp = datetime.now().strftime('%d-%m-%Y %H-%M-%S')
        os.makedirs(self.path, exist_ok=True)
        self.name = os.path.join(self.path, f"EIT - {self.timestamp}.db")
        print(f"Logging: {self.name}")
        self.conn = sqlite3.connect(self.name)
        self.cursor = self.conn.cursor()

        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS measurements (
            timestamp TEXT,
            data TEXT
        )
        """)

        self.conn.commit()
        self.isRunning = True

    def stopLog(self):
        if self.isRunning:
            self.conn.close()
            self.isRunning = False
