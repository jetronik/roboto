from agvcloud import AGVcloud


if __name__ == "__main__":

    # THX variables. Can be pass as arguments in command line if suitable
    appKey = '93c04eda-3112-4b53-a94b-0060a9e05fe7'
    thing_name = 'AGVtest'

    # Initialize THX handler
    h = AGVcloud(appKey)

    # Simple GET (read) operation
    #[resp, data] = h.get_thing_property('AGVtest', 'TestGet', print_result=True)

    # Simple PUT (write) operation: matches data from TestGet
    #resp = h.put_thing_property('VictorAGVtest', 'TestPut', data, print_result=True, show_old_value=True)

    # Simple GET (read) operation wrapped in try/except. 
    # NOT RECOMENDED since most typical exceptions should be already cover by handlers.
    #try:
    #    [resp, data] = h.get_thing_property('VictorAGVtest', 'TestGet', print_result=True)
    #except:
    #    print("Something went wrong!")

    # GET and search in infotable (tabular data)
    [resp, data] = h.get_thing_property(thing_name, 'Goals')
    # filtered_data = h.search_in_infotable(data, 'GoalName', 'UR5', return_column='GoalId', print_result=True)
    filtered_data = h.search_in_infotable(data, 'GoalName', 'UR10', return_column='GoalPosition', print_result=True)

    print(data)

    test={[0,0,0,0]}



    #resp = h.put_thing_property(thing_name, 'TestPut', test, print_result=True, show_old_value=True)


    # GET complete response from THX.
    # NOT RECOMENDED unless advanced use is required.
    # [resp, data] = h.get_all_thing_properties('VictorAGVtest', print_result=True, raw=True)
