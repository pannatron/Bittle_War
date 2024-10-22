import asyncio
from bleak import BleakClient

uuid = "661B09E8-8866-A3E9-F1EE-0008208263E7"

async def discover_services_and_characteristics():
    async with BleakClient(uuid) as client:
        if client.is_connected:
            print("เชื่อมต่อกับ Petoi สำเร็จ")
            for service in client.services:
                print(f"Service: {service.uuid}")
                for char in service.characteristics:
                    print(f"  Characteristic: {char.uuid}, (Handle: {char.handle})")
        else:
            print("ไม่สามารถเชื่อมต่อกับ Petoi")

asyncio.run(discover_services_and_characteristics())
