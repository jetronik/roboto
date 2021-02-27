import requests
import json
import sys
from apierror import APIError


class AGVcloud(object):
    """Handler class to operate AGV through Thingworx using REST API.
    Thing names and property names are case sensitive"""
    def __init__(self, appKey):
        self.__appKey = appKey
        self.__THX_URL = 'https://assar.his.se/Thingworx/Things/'
        self.__headers_get = {'Accept':'application/json', "appKey": self.__appKey}
        self.__headers_put = {'Content-Type':'application/json', "appKey": self.__appKey}

    def get_all_thing_properties(self, thing_name, print_result=False, raw=False):
        """Get all the properties of a thing

        Keyword arguments:
        thing_name      -- thing name on the cloud [string]
        raw             -- when true, gets metadata [bool]
        print_result    -- prints before return [bool]

        return          -- response from THX [JSON]
        """

        url = self.__THX_URL + thing_name + '/Properties'
        resp = requests.get(url, headers=self.__headers_get)

        if resp.status_code != 200:
            raise APIError('GET', resp.status_code)
        else:
            if raw:
                if print_result:
                    print(resp.json())
                return(resp, resp.json())
            else:
                if print_result:
                    print(resp.json()['rows'][0])
                return(resp, resp.json()['rows'][0])

    def get_thing_property(self, thing_name, property_name, print_result=False):
        """Get pointed property of a thing

        Keyword arguments:
        thing_name      -- thing name on the cloud [string]
        property_name   -- property name on the cloud [string]
        print_result    -- prints before return [bool]

        return          -- value of property [equivalent from THX]
        """

        url = self.__THX_URL + thing_name + '/Properties/' + property_name
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
        print_result    -- prints before return [bool]
        show_old_value  -- prints new and old values [bool]

        return          -- ok response (200) [string]
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

    def search_in_infotable(self, infotable, matching_column, matching_value, return_column=None, print_result=False):
        """Handler method to extract info from datatables

        Keyword arguments:
        infotable       -- raw data from GET request [JSON]
        matching_column -- column/field name in THX to find [string]
        matching_value  -- value to match [variable, same type as in JSON]
        return_column   -- column/field name in THX to return [string]. None = return entire row
        print_result    -- prints before return [bool]

        return          -- matching value from column or return column [equivalent from THX alone or in list]
        """
        # Tries to adapt to infotable shape
        if 'rows' in infotable:
            rows_infotable = infotable['rows']
        else:
            rows_infotable = infotable

        try:
            filtered_data = list(filter(lambda _infotable: _infotable[matching_column] == matching_value, rows_infotable))
        except KeyError as e:
            if sys.version_info[0] < 3:
                raise KeyError('Matching column %s not found. Check THX infotable' % matching_column)
            else:
                raise KeyError('Matching column %s not found. Check THX infotable' % matching_column) 

        return_data = []
        if return_column:
            try:
                for row in range(0,len(filtered_data)):
                    return_data.append(filtered_data[row][return_column])
                # If only one value, do not return list.
                if len(filtered_data) == 1:
                    return_data = return_data[0]
            except KeyError as e:
                if sys.version_info[0] < 3:
                    raise KeyError('Return column %s not found. Check THX infotable' % return_column)
                else:
                    raise KeyError('Return column %s not found. Check THX infotable' % return_column) 
        else:
            return_data = filtered_data

        if print_result:
            print(return_data)

        if return_data == []:
            print('Warning! No match in infotable')

        return return_data

    def get_infotable(self, thing_name, infotable_name, matching_column, matching_value, return_column=None, print_result=False):
        """Handler method to extract info from datatables. Includes get request.
        This method is a compacted version of search in infotable

        Keyword arguments:
        thing_name      -- thing name on the cloud [string]
        infotable_name  -- infortable name on the cloud [string]
        matching_column -- column/field name in THX to find [string]
        matching_value  -- value to match [variable, same type as in JSON]
        return_column   -- column/field name in THX to return [string]. None = return entire row
        print_result    -- prints before return [bool]

        return          -- matching value from column or return column [equivalent from THX alone or in list]
        """
        # Tries to adapt to infotable shape
        [_, data] = self.get_thing_property(thing_name, infotable_name)

        if 'rows' in data:
            rows_data = data['rows']
        else:
            rows_data = data

        try:
            filtered_data = list(filter(lambda _infotable: _infotable[matching_column] == matching_value, rows_data))
        except KeyError as e:
            raise KeyError('Matching column %s not found. Check THX infotable' % matching_column) 

        return_data = []
        if return_column:
            try:
                for row in range(0,len(filtered_data)):
                    return_data.append(filtered_data[row][return_column])
                # If only one value, do not return list.
                if len(filtered_data) == 1:
                    return_data = return_data[0]
            except KeyError as e:
                if sys.version_info[0] < 3:
                    raise KeyError('Return column %s not found. Check THX infotable' % return_column)
                else:
                    raise KeyError('Return column %s not found. Check THX infotable' % return_column) 
        else:
            return_data = filtered_data

        if print_result:
            print(return_data)

        if return_data == []:
            print('Warning! No match in infotable')

        return return_data

    def put_in_infotable(self, thing_name, infotable_name, matching_column, matching_value, target_column, target_value, 
        print_result=False, show_old_value=False):
        """Put pointed value on pointed infotable of a thing

        Keyword arguments:
        thing_name      -- thing name on the cloud [string]
        infotable_name  -- infortable name on the cloud [string]
        matching_column -- column/field name in THX to find [string]
        matching_value  -- value to match [variable, same type as in JSON]
        target_column   -- column/field name in THX to overwrite [string]
        target_value    -- value to put target column match [variable, same type as in JSON]
        print_result    -- prints before return [bool]
        show_old_value  -- prints new and old values [bool]

        return          -- ok response (200) [string]
        """

        [_, data] = self.get_thing_property(thing_name, infotable_name)
        #print(data['rows'])
        if show_old_value:
            old_value = self.search_in_infotable(data['rows'], matching_column, matching_value, return_column=target_column)
            #print(old_value)
        # Check there ir only one matching value. Otherwise, warnning is given
        #if len(old_value) > 1:
        #    old_value = old_value
        #    print('[WARN] More than one match! Only handling first appereance.')

        # Finds index of the first appereance
        idx = next((index for (index, d) in enumerate(data['rows']) if d[matching_column] == matching_value), None)
        # Overwrites data
        data['rows'][idx][target_column] = target_value

        url = self.__THX_URL + thing_name + '/Properties/' + infotable_name
        resp = requests.put(url, data=json.dumps({infotable_name: data}), headers=self.__headers_put)

        if resp.status_code != 200:
            raise APIError('GET', resp.status_code)
        else:
            if print_result:
                if show_old_value:
                    print("THX %s[%s] set from %s to %s" % (thing_name, infotable_name, old_value, target_value))
                else:
                    print("THX %s[%s] set to %s" % (thing_name, infotable_name, target_value))
        
        return resp