import { configureStore } from "@reduxjs/toolkit";
import robotInfoReducer from "./RobotInfo";
// import expenseReducer from "./ExpenseSlice";
// import pageReducer from "./PageSlice";
import actionLockReducer from "./ActionLock";
import snackbarReducer from "./SnackbarState";
// Info source: https://www.youtube.com/watch?v=_shA5Xwe8_4
export const store = configureStore({
	reducer: {
		robotInfo: robotInfoReducer,
		snackbar: snackbarReducer,
		actionLock: actionLockReducer,
		// expenses: expenseReducer,
		// pages: pageReducer,
	},
});

export type RootState = ReturnType<typeof store.getState>;
