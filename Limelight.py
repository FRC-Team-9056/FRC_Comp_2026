import ntcore
import time

def main():
    # Grab the default NT instance
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("apriltag-reader")
    inst.setServerTeam(9056)  # <-- update this
    inst.startDSClient()  # Optional if running on coprocessor

    # Limelight table
    ll = inst.getTable("limelight")

    # Your output table
    tagsTable = inst.getTable("apriltags")
    pubTags = tagsTable.getIntegerArrayTopic("tags").publish()

    print("AprilTag reader running (Limelight mode)...")

    while True:
        # Limelight botpose (6- or 7-element array)
        botpose = ll.getEntry("botpose").getDoubleArray([])

        # ID of closest detected tag
        tid = int(ll.getEntry("tid").getDouble(-1))

        detected_ids = []

        if tid != -1:
            detected_ids.append(tid)

            # Publish pose under your format
            if len(botpose) >= 6:
                tagsTable.getEntry(f"pose_{tid}").setDoubleArray(
                    [botpose[0], botpose[1], botpose[2], botpose[3], botpose[4], botpose[5]]
                )

        # Publish ID list
        pubTags.set(detected_ids)

        time.sleep(0.02)  # 50Hz loop
