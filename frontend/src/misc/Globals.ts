export default class Globals {
	public static readonly BG_SURFACE_CONTAINER = "bg-current/5";

	public static getRobotAddr() {
		return import.meta.env.VITE_COMMANDER_ADDR ?? "localhost:5006";
	}
}
