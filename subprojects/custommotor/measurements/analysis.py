# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.13.7
#   kernelspec:
#     display_name: Python 3 (ipykernel)
#     language: python
#     name: python3
# ---

import pandas as pd
df = pd.read_csv('nocontroller.csv', names=['timestamp', 'ticks'])
df['timestamp'] = pd.to_datetime(df['timestamp'])
df['freq'] = (12E6/(df['ticks']))*60
df = df[df['freq'] < 30000]

df.head()

df.plot(x='timestamp', y='freq',
        xlabel='Time [day, hours, minutes]',
        ylabel='Frequency [rpm]')

