from live_advance import LiveAdvance

CLIENT_SECRET = 'your emotiv client secret'
CLIENT_ID = 'your emotiv client ID'

def main():
    workflow = LiveAdvance(app_client_id=CLIENT_ID, app_client_secret=CLIENT_SECRET)
    workflow.start("your emotiv profile name")

if __name__ == "__main__":
    main()