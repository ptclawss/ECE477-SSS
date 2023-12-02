# from django.shortcuts import render
# from django.http import HttpResponse
# import json

# # Create your views here.
# from .models import Seat
# import json

# fsr1_thresh = 2000
# fsr2_thresh = 2000
# AMG8833_thresh = 20

# def seat_status(request):
#     if request.method == 'POST':
#         try:

#             # Decode the JSON data from the request body
#             json_str = request.body.decode('utf-8')
#             json_obj = json.loads(json_str)
#             print(json_obj)

#             # You can also save the received data to the database or perform other actions as needed
#             occupancy = True
#             if (float(json_obj['FSR1']) >= float(fsr1_thresh) or float(json_obj['FSR2']) >= float(fsr2_thresh)) and (float(json_obj['AMG8833']) >= float(AMG8833_thresh)):
#                 occupancy = True
#             else:
#                 occupanchy = False

#             print(occupanchy)


#             return HttpResponse("POST request received successfully.")
#         except json.JSONDecodeError as e:
#             print("Error decoding JSON:", str(e))
#             return HttpResponse("Error decoding JSON.", status=400)

#     else:
#         # For other HTTP methods or when accessing the view via a browser
#         seats_data = Seat.objects.all()
#         context = {'seats_data': seats_data}
#         return render(request, 'seat_app/deploy.html', context)
    
from django.shortcuts import render
from django.http import HttpResponse
import json

# Create your views here.
from .models import Seat
import json

fsr1_thresh = 2000
fsr2_thresh = 2000
AMG8833_thresh = 20

def seat_status(request):
    if request.method == 'POST':
        try:
            # Decode the JSON data from the request body
            json_str = request.body.decode('utf-8')
            json_obj = json.loads(json_str)

            # You can also save the received data to the database or perform other actions as needed
            occupancy = True
            if (float(json_obj['FSR1']) >= float(fsr1_thresh) or float(json_obj['FSR2']) >= float(fsr2_thresh)) and (float(json_obj['AMG8833']) >= float(AMG8833_thresh)):
                occupancy = True
            else:
                occupancy = False

            print(occupancy)

            # Update the output.json file based on the received sensor values
            update_json_file(json_obj['SeatID'], occupancy)

            return HttpResponse("POST request received successfully.")
        except json.JSONDecodeError as e:
            print("Error decoding JSON:", str(e))
            return HttpResponse("Error decoding JSON.", status=400)

    else:
        # For other HTTP methods or when accessing the view via a browser
        seats_data = Seat.objects.all()
        context = {'seats_data': seats_data}
        return render(request, 'seat_app/deploy.html', context)

def update_json_file(seat_id, occupancy):
    # Load existings data from the output.json file
    with open('C:\\Users\\roshs\\Downloads\\front_end_Roshan\\testproject_-_Copy\\testproject - Copy\\seat_app\\static\\seat_app\\output.json', 'r') as file:
        existing_data = json.load(file)

    # Set "A1" occupancy to True
    for row_data in existing_data:
        if row_data["row"] == seat_id[0]:
            for seat in row_data["seats"]:
                if seat["seat_id"] == seat_id:
                    seat["occupancy"] = occupancy
                    break

    # Convert the modified data back to JSON
    #modified_json_data = json.dumps(data, indent=2)

# Print the result
#print("Occupancy of A1:", a1_occupancy)

    # Save the updated data back to the output.json file
    with open('C:\\Users\\roshs\\Downloads\\front_end_Roshan\\testproject_-_Copy\\testproject - Copy\\seat_app\\static\\seat_app\\output.json', 'w') as file:
        json.dump(existing_data, file, indent=2)
