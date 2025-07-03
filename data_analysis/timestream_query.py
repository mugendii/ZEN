import boto3
import pandas as pd

# Query Timestream
client = boto3.client('timestream-query', region_name='us-east-1')

query = """
SELECT 
  BIN(time, 1m) AS binned_time,
  deviceId,
  sensorType,
  measure_name,
  AVG(
    COALESCE(
      measure_value::double,
      CAST(measure_value::bigint AS double)
    )
  ) AS avg_value
FROM "IoTdatabase"."SensorData"
WHERE measure_name IN ('co2', 'pressure', 'temperature')
GROUP BY BIN(time, 1m), deviceId, sensorType, measure_name
ORDER BY binned_time DESC
"""

response = client.query(QueryString=query)

# Parse results
column_info = response['ColumnInfo']
rows = response['Rows']

def parse_row(row):
    return [d.get('ScalarValue', None) for d in row['Data']]

data = [parse_row(row) for row in rows]
columns = [col['Name'] for col in column_info]
df = pd.DataFrame(data, columns=columns)

# Convert to proper types
df['binned_time'] = pd.to_datetime(df['binned_time'])


pivot_df = df.pivot_table(index=['binned_time', 'deviceId'], 
                          columns='measure_name', 
                          values='avg_value',
                          aggfunc='first').reset_index()

# Export to CSV
pivot_df.to_csv('sensor_data.csv', index=False)
print("Pivoted data exported to 'sensor_data.csv'")
