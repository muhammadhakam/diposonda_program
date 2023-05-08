import pandas as pd
import time

while True:
    # Get real-time data
    data = {'name': ['John', 'Mike', 'Sarah'], 'age': [25, 30, 28], 'city': ['New York', 'Los Angeles', 'Chicago']}
    df = pd.DataFrame(data)

    # Write data to CSV file
    with open('output.csv', 'a') as f:
        df.to_csv(f, header=f.tell()==0, index=False)

    # Wait for 1 second
    time.sleep(1)