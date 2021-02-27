#!/usr/bin/env python


class APIError(Exception):
    """An API Error Exception"""

    def __init__(self, operation, status):
        self.operation = operation
        self.status = status
        self.error_msg = ErrorMessages(operation, status).checkResponse(status)

    def __str__(self):
        return ("APIError: {}".format(self.error_msg)) 


class ErrorMessages:
    def __init__(self, operation, status):
        self.operation = operation
        self.status = status
        self.checkResponse(status)

    def checkResponse(self, status_code):
        """Dispatch method"""
        error_num = 'error_' + str(status_code)
        method = getattr(self, error_num, lambda: "Unexpected {} request".format(self.operation) 
            + " returned error {}".format(self.status))
        return method()

    def error_401(self):
        return("Error 401 - unauthorized: appKey is incorrect or missing")
 
    def error_403(self):
        return ("Error 403 - forbidden: Content-Type request header is not set to application/json. " + 
            "Sometimes returned instead of a 404. " + 
            "A Property with that name already exists on the platform")
    
    def error_404(self):
        return ("Error 404 - not found: Incorrect URL or API endpoint. " + 
            "Thing or Property has not been created. " + 
            "Incorrect ThingTemplate name. " + 
            "Required parameter missing from request")
    
    def error_405(self):
        return ("Error 405 - invalid_request: Incorrect request verb; for example a GET was used instead of PUT or POST")
    
    def error_406(self):
        return ("Error 406 - not_acceptable: Invalid JSON in PUT or POST request. " + 
            "Thing [ Thing name ] already exists: A Thing with that name already exists on the platform")
    
    def error_500(self):
        return ("Error 500 - internal_server_error: Content-Type request header is not set for a service execution POST, " + 
            "required even without a POST body Content-Type request header is not set for DELETE request, " + 
            "required despite the fact that DELETE does not send any content")
    
    def error_503(self):
        return ("Error 503 - service_unavailable: Thing [] is not running. RestartThing endpoint must be called. " + 
        "Thing [] is not enabled EnableThing endpoint must be called")
