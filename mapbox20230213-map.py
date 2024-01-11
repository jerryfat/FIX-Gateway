

#token = open(".mapbox_token").read() # you will need your own token
token =  'pk.eyJ1IjoiamVycnlmYXQiLCJhIjoiY2tjczFwZnp0MDJuaDJ6cWpkZHF5YjM1cCJ9.8pwSnCZUuvsm8Tzm_LYt4g'
import pandas as pd
us_cities = pd.read_csv("https://raw.githubusercontent.com/plotly/datasets/master/us-cities-top-1k.csv")

import plotly.express as px

fig = px.scatter_mapbox(us_cities, lat="lat", lon="lon", hover_name="City", hover_data=["State", "Population"],
                        color_discrete_sequence=["fuchsia"], zoom=3, height=300)
fig.update_layout(mapbox_style="dark", mapbox_accesstoken=token)
fig.update_layout(margin={"r":0,"t":0,"l":0,"b":0})
fig.show()


