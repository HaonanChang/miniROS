import Counter from "../components/Counter";
import GelloAlignWidget from "../components/GelloStatusWidget";
import RobotStatusWidget from "../components/RobotStatusWidget";

export default function NormalPanel() {
	return (
		<div className="flex flex-col gap-y-5">
			<div className="flex flex-row items-center justify-center">
				<Counter />
			</div>
			<RobotStatusWidget />
			<GelloAlignWidget />
		</div>
	);
}
