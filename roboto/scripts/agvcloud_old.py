import requests
import json
from apierror import APIError


class AGVcloud(object):
    """Handler class to operate AGV through Thingworx using REST API.
    Thing names and property names are case sensitive"""

    def __init__(self, appKey):
        self.__THX_URL = 'https://assar.his.se/Thingworx/Things/'
        self.__appKey = appKey
        self.__headers_get = {'Accept':'application/json', "appKey": self.__appKey}
        self.__headers_put = {'Content-Type':'application/json', "appKey": self.__appKey}

    def get_all_thing_properties(self, thing_name, raw=False):
        """Get all the properties of a thing

        Keyword arguments:
        thing_name      -- thing name on the cloud [string]
        raw             -- when true, gets metadata [bool] (default false)
        """

        url = self.__THX_URL + thing_name + '/Properties'
        resp = requests.get(url, headers=self.__headers_get)

        if resp.status_code != 200:
            raise APIError('GET', resp.status_code)
        else:
            if raw:
                return(resp, resp.json())
            else:
                return(resp, resp.json()['rows'][0])

    def get_thing_property(self, thing_name, property_name, print_result=False):
        """Get pointed property of a thing

        Keyword arguments:
        thing_name      -- thing name on the cloud [string]
        property_name   -- property name on the cloud [dynamic]
        """

        url = self.__THX_URL + thing_name + '/Properties'
        resp = requests.get(url, headers=self.__headers_get)

        if resp.status_code != 200:
            raise APIError('GET', resp.status_code)
        else:
            if print_result:
                print("THX %s[%s] = %s" % (thing_name, property_name, 
                    resp.json()['rows'][0][property_name]))
            return(resp, resp.json()['rows'][0][property_name])

    def put_thing_property(self, thing_name, property_name, value, print_result=False, show_old_value=False):
        """Put pointed value on pointed property of a thing

        Keyword arguments:
        thing_name      -- thing name on the cloud [string]
        property_name   -- property name on the cloud [dynamic]
        value           -- value to put on cloud
        """
        if show_old_value:
            [_, old_value] = self.get_thing_property(thing_name, property_name)

        url = self.__THX_URL + thing_name + '/Properties/' + property_name
        resp = requests.put(url, data=json.dumps({property_name: value}), headers=self.__headers_put)

        if resp.status_code != 200:
            raise APIError('GET', resp.status_code)
        else:
            if print_result:
                if show_old_value:
                    print("THX %s[%s] set from %s to %s" % (thing_name, property_name, old_value, value))
                else:
                    print("THX %s[%s] set to %s" % (thing_name, property_name, value))
            return resp
