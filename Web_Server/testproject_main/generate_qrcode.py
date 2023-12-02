import qrcode

# Replace 'https://example.com' with your desired URL
url = 'http://10.0.0.247:8000/seat_app/seat_status/'

# Generate QR code
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_L,
    box_size=10,
    border=4,
)
qr.add_data(url)
qr.make(fit=True)

# Create an image from the QR code instance
img = qr.make_image(fill_color="black", back_color="white")

# Save the image to a file
img.save('qrcode.png')
