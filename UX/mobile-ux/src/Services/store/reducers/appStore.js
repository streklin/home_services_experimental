import * as actionTypes from '../actions';

const initialState = {
    isMenuOpen: false,
    token: null,
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

const setToken = (state, action) => {
    let newState = Object.assign({}, state);
    newState.token = action.data;
    return newState;
};

const clearToken = (state, action) => {
    let newState = Object.assign({}, state);
    newState.token = null;
    return newState;
};

const enableAutoMap = (state, action) => {
    let newState = Object.assign({}, state);
    newState.robotState.isAutoMapActive = true;
    return newState;
};

const disableAutoMap = (state, action) => {
    let newState = Object.assign({}, state);
    newState.robotState.isAutoMapActive = false;
    return newState;
};

const appReducerObj = {};

appReducerObj[actionTypes.TOGGLE_SIDE_MENU] = toggleSideMenu;
appReducerObj[actionTypes.UPDATE_SENSOR_STATES] = updateSensorStates;
appReducerObj[actionTypes.UPDATE_CAMERA_URL] = updateCamUrl;
appReducerObj[actionTypes.UPDATE_MAP_URL] = updateMapUrl;
appReducerObj[actionTypes.UPDATE_CONVERSATION] = updateConversation;
appReducerObj[actionTypes.SHOW_ERROR_MODAL] = showErrorModal;
appReducerObj[actionTypes.LOCK_CHAT] = lockChat;
appReducerObj[actionTypes.UNLOCK_CHAT] = unlockChat;
appReducerObj[actionTypes.SET_LOGIN_TOKEN] = setToken;
appReducerObj[actionTypes.CLEAR_LOGIN_TOKEN] = clearToken;
appReducerObj[actionTypes.ENABLE_AUTOMAP] = enableAutoMap;
appReducerObj[actionTypes.DISABLE_AUTOMAP] = disableAutoMap;


const reducer = (state = initialState, action) => {

    if (appReducerObj.hasOwnProperty(action.type)) {
        return appReducerObj[action.type](state, action);
    }


    return state;
};

export default reducer;