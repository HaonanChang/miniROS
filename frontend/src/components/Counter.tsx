import RestartAltIcon from "@mui/icons-material/RestartAlt";
import { Button } from "@mui/material";
import { useTranslation } from "react-i18next";
import { useDispatch, useSelector } from "react-redux";
import RobotRequest from "../backend/RobotRequest";
import Globals from "../misc/Globals";
import { setSnackBar, SnackbarType } from "../redux/SnackbarState";
import { RootState } from "../redux/Store";

export default function Counter() {
	const { t } = useTranslation();

	const value = useSelector((state: RootState) => state.robotInfo.counter);
	const dispatch = useDispatch();

	const handleReset = async () => {
		try {
			await RobotRequest.resetCounter();

			dispatch(
				setSnackBar({
					message: t("episode_counter/prompt_reset"),
					type: SnackbarType.GOOD,
					durationMs: 1500,
				}),
			);
		} catch (e: unknown) {
			dispatch(
				setSnackBar({
					message: "Failed to reset counter! " + (e as Error).message,
					type: SnackbarType.ERROR,
					durationMs: 1500,
				}),
			);
		}
	};

	return (
		<div
			className={`flex flex-row rounded-2xl ${Globals.BG_SURFACE_CONTAINER} h-min items-center justify-center`}
		>
			<div className="px-5">
				<p className="text-lg text-black dark:text-white font-bold">
					{t("episode_counter/count")}: {value}
				</p>
			</div>
			<Button
				variant="contained"
				color="error"
				sx={{
					borderRadius: "16px",
					textTransform: "none",
				}}
				disabled={value === 0}
				onClick={handleReset}
			>
				<div className="flex flex-row items-center justify-center gap-x-1">
					<RestartAltIcon />
					<p className="text-lg">{t("episode_counter/reset")}</p>
				</div>
			</Button>
		</div>
	);
}
