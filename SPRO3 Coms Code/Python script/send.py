import csv
import requests

mkr_ip = "192.168.x.x"

csv_file_path = 'trash.csv'

# Function to read the last row of the CSV file
def read_last_row(csv_filename):
    with open(csv_filename, 'r', newline='') as csvfile:
        data_reader = csv.reader(csvfile)
        last_row = None
        for row in data_reader:
            if row:  # skip empty rows
                last_row = row
        return last_row

# Read the last row from the CSV file
last_row = read_last_row(csv_file_path)
if last_row and len(last_row) >= 2:
    value1 = last_row[-2]  # Second to last value
    value2 = last_row[-1]  # Last value

    # Construct the URL with parameters
    url = f"http://{mkr_ip}/send?value1={value1}&value2={value2}"

    # Send the GET request
    response = requests.get(url)

    # Print the response
    print(response.text)
else:
    print("CSV file does not contain enough data.")
