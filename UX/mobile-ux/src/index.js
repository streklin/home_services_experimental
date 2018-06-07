import React from 'react';
import ReactDOM from 'react-dom';
import './index.css';
import App from './Views/App/App';
import registerServiceWorker from './registerServiceWorker';
import { createStore, combineReducers, applyMiddleware} from 'redux';
import thunk from 'redux-thunk';
import { Provider } from 'react-redux';
import appReducer from './Services/store/reducers/appStore';

const rootReducer = combineReducers({
   appStore: appReducer
});

const store = createStore(rootReducer, {}, applyMiddleware(thunk));

ReactDOM.render(<Provider store={store}><App /></Provider>, document.getElementById('root'));
registerServiceWorker();
