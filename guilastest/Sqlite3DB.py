import sqlite3
import os

class Sqlite3DB:
    def __init__(self, db_name, all_values=None):
        try:
            self.conn = sqlite3.connect(db_name)
            self.cursor = self.conn.cursor()
            if all_values:
                self.all_values = all_values
            else:
                self.all_values = ['date', 'time', 'PR', 'SpO2', 'Temp', 'Sys', 'Dia']
        except sqlite3.Error as e:
            print(f"Error connecting to database: {e}")
            raise

    def create_table(self, room_id):
        try:
            execute_string = f'CREATE TABLE IF NOT EXISTS {room_id} (' + ', '.join([f'{value} INTEGER' for value in self.all_values]) + ')'
            self.cursor.execute(execute_string)
            self.conn.commit()
            print(f"Table {room_id} created successfully.")
        except sqlite3.Error as e:
            print(f"Error creating table {room_id}: {e}")
            self.conn.rollback()

    def insert_data(self, room_id, data):
        try:
            execute_string = f'INSERT INTO {room_id} VALUES (' + ', '.join(['?' for _ in range(len(self.all_values))]) + ')'
            self.cursor.execute(execute_string, data)
            self.conn.commit()
            print("Data inserted successfully.")
        except sqlite3.Error as e:
            print(f"Error inserting data into {room_id}: {e}")
            self.conn.rollback()

    def select_data(self, room_id):
        try:
            self.cursor.execute(f'SELECT * FROM {room_id}')
            val = self.cursor.fetchall()
            res = [dict(zip(self.all_values, row)) for row in val]
            return res
        except sqlite3.Error as e:
            print(f"Error selecting data from {room_id}: {e}")
            return []

    def close(self):
        try:
            self.conn.close()
            print("Database connection closed.")
        except sqlite3.Error as e:
            print(f"Error closing database connection: {e}")

    def delete_table(self, room_id):
        try:
            self.cursor.execute(f'DROP TABLE IF EXISTS {room_id}')
            self.conn.commit()
            print(f"Table {room_id} deleted successfully.")
        except sqlite3.Error as e:
            print(f"Error deleting table {room_id}: {e}")
            self.conn.rollback()

    def show_tables(self):
        try:
            self.cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
            val = self.cursor.fetchall()
            return [table[0] for table in val]
        except sqlite3.Error as e:
            print(f"Error fetching table list: {e}")
            return []

# Example usage:
# db_path = os.path.join(os.path.expanduser("~"), "Desktop", "Database", "your_database.db")
# db = Sqlite3DB(db_path)
# db.create_table('room_001')
# db.create_table('room_002')
# data = ['2023-10-13', '12:00:00', 80, 98, 36.5, 120, 80]
# db.insert_data('room_001', data)
# print(db.select_data('room_001'))
# print(db.show_tables())
# db.close()
