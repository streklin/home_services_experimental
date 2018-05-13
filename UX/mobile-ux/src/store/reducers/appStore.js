import * as actionTypes from '../actions';

const initialState = {
    isMenuOpen: false,
    mapUrl: null,
    camUrl: null,
    showError: false,
    conversation: [],
    isChatLocked: false,
    robotState: {
        isAutoMapActive: false,
        leftBumperPressed: false,
        rightBumperPressed: false,
        wheelsDropped: false,
        spotLightOn: false,
        debrisLightOn: false,
        isLeftSensorActive: false,
        isFrontLeftSensorActive: false,
        isCenterLeftSensorActive: false,
        isCenterRightSensorActive: false,
        isFrontRightSensorActive: false,
        isRightSensorActive: false,
        batteryPower: 0.0
    }
};

const toggleSideMenu = (state) => {
    let newState = Object.assign({}, state);
    newState.isMenuOpen = !newState.isMenuOpen;
    return newState;
};

const toggleAutoMap = (state) => {
    let newState = Object.assign({}, state);
    newState.robotState.isAutoMapActive = !newState.robotState.isAutoMapActive;
    return newState;
};

const updateSensorStates = (state, action) => {
    let data = action.data;

    let newState = Object.assign({}, state);
    newState.robotState = {
        isAutoMapActive: newState.isAutoMapActive,
        leftBumperPressed: data.leftBumperPressed,
        rightBumperPressed: data.rightBumperPressed,
        wheelsDropped: data.wheelsDropped,
        spotLightOn: data.spotLightOn,
        debrisLightOn: data.debrisLightOn,
        isLeftSensorActive: data.isLeftSensorActive,
        isFrontLeftSensorActive: data.isFrontLeftSensorActive,
        isCenterLeftSensorActive: data.isCenterLeftSensorActive,
        isCenterRightSensorActive: data.isCenterRightSensorActive,
        isFrontRightSensorActive: data.isFrontRightSensorActive,
        isRightSensorActive: data.isRightSensorActive,
        batteryPower: data.batteryPower
    };

    return newState;
};

const updateMapUrl = (state, action) => {
    let newState = Object.assign({}, state);
    newState.mapUrl = action.data;
    return newState;
};

const updateCamUrl = (state, action) => {
    let newState = Object.assign({}, state);
    newState.camUrl = action.data;
    return newState;
};

const updateConversation = (state, action) => {
    let newState = Object.assign({}, state);

    newState.conversation = newState.conversation.map((element) => {
        return element;
    });
    newState.conversation.push(action.data);

    return newState;
};

const showErrorModal = (state, action) => {
    let newState = Object.assign({}, state);

    newState.showError = true;

    return newState;
};

const lockChat = (state) => {

    let newState = Object.assign({}, state);
    newState.isChatLocked = true;
    return newState;

};

const unlockChat = (state) => {

    let newState = Object.assign({}, state);
    newState.isChatLocked = false;
    return newState;

};

const reducer = (state = initialState, action) => {

    switch(action.type) {
        case actionTypes.TOGGLE_SIDE_MENU:
            return toggleSideMenu(state);
        case actionTypes.TOGGLE_AUTO_MAP:
            return toggleAutoMap(state);
        case actionTypes.UPDATE_SENSOR_STATES:
            return updateSensorStates(state, action);
        case actionTypes.UPDATE_CAMERA_URL:
            return updateCamUrl(state, action);
        case actionTypes.UPDATE_MAP_URL:
            return updateMapUrl(state, action);
        case actionTypes.UPDATE_CONVERSATION:
            return updateConversation(state, action);
        case actionTypes.SHOW_ERROR_MODAL:
            return showErrorModal(state, action);
        case actionTypes.LOCK_CHAT:
            return lockChat(state);
        case actionTypes.UNLOCK_CHAT:
            return unlockChat(state);
        default:
            return state;
    }

};

export default reducer;