from live_advance import LiveAdvance

CLIENT_SECRET = 'DSveuaZepqJFlJakhsBTluKa6iU1vDkLvO9wo0ZwBWsIux4eoHRXypl6j6O1qgd5fXCMdt8ZAH7Vym28yYYz7ydtGsjIyitzL5VGYfSJw1u9QyOj5mqJwrVBoHGNAay4'
CLIENT_ID = 'mQOlnlVpAG5F0tyN0QTHZT5l1HqGQlDrCqHcbOnF'

def main():
    workflow = LiveAdvance(app_client_id=CLIENT_ID, app_client_secret=CLIENT_SECRET)
    workflow.start("teste")

if __name__ == "__main__":
    main()